use crate::config::ParameterConfigColor;
use crate::{config::ListenerConfigColor, transformation::ros_transform_to_isometry};
use nalgebra::Point3;
use rustros_tf;
use std::sync::{Arc, RwLock};
use tui::style::Color;
use tui::widgets::canvas::Line;

pub fn get_polygon_from_ros_param(param: &String) -> Option<rosrust_msg::geometry_msgs::Polygon> {
    let ros_param_polygon = rosrust::param(param);
    if let Some(ros_param_polygon) = &ros_param_polygon {
        let mut polygon = rosrust_msg::geometry_msgs::Polygon { points: vec![] };
        let fb = ros_param_polygon.get::<Vec<Vec<f64>>>();
        match fb {
            Ok(f) => {
                for pt in f {
                    polygon.points.push(rosrust_msg::geometry_msgs::Point32 {
                        x: pt[0] as f32,
                        y: pt[1] as f32,
                        z: 0 as f32,
                    });
                }
                return Some(polygon);
            }
            Err(_e) => {
                println!("{} not found, not showing polygon.", param);
                return None;
            }
        }
    }
    return None;
}

pub fn read_points(msg: &rosrust_msg::geometry_msgs::Polygon) -> Vec<Point3<f64>> {
    let n_pts = msg.points.len();
    let mut points: Vec<Point3<f64>> = Vec::with_capacity(n_pts as usize);
    for pt in msg.points.iter() {
        points.push(Point3::new(pt.x as f64, pt.y as f64, pt.z as f64));
    }
    return points;
}

pub struct PolygonData {
    pub polygon_stamped_msg: Option<rosrust_msg::geometry_msgs::PolygonStamped>,
    pub lines_in_static_frame: Option<Vec<Line>>,
    _color: Color,
    _tf_listener: Arc<rustros_tf::TfListener>,
    _static_frame: String,
}

pub struct PolygonListener {
    _data: Arc<RwLock<PolygonData>>,
    _subscriber: rosrust::Subscriber,
}

pub struct ParameterPolygon {
    _data: Arc<RwLock<PolygonData>>,
}

impl PolygonData {
    pub fn update(&mut self) {
        self.lines_in_static_frame = None;
        if let Some(polygon) = &self.polygon_stamped_msg {
            let transform = self._tf_listener.clone().lookup_transform(
                &self._static_frame.clone(),
                &polygon.header.frame_id,
                polygon.header.stamp,
            );

            match &transform {
                Ok(transform) => {
                    let mut lines: Vec<Line> = Vec::new();
                    let polygon_transform = ros_transform_to_isometry(&transform.transform);
                    let mut previous_point: Option<Point3<f64>> = None;
                    for pt in read_points(&polygon.polygon) {
                        let point_in_static_frame = polygon_transform.transform_point(&pt);
                        if let Some(pp) = previous_point {
                            lines.push(Line {
                                x1: pp.x,
                                y1: pp.y,
                                x2: point_in_static_frame.x,
                                y2: point_in_static_frame.y,
                                color: self._color.clone(),
                            });
                        }
                        previous_point = Some(point_in_static_frame);
                    }

                    self.lines_in_static_frame = Some(lines);
                }
                Err(_e) => {}
            };
        }
    }

    pub fn get_lines(&self) -> Vec<Line> {
        if let Some(lines) = &self.lines_in_static_frame {
            return lines.to_vec();
        }
        return vec![];
    }
}

impl ParameterPolygon {
    pub fn new(
        config: ParameterConfigColor,
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
    ) -> ParameterPolygon {
        let data = Arc::new(RwLock::new(PolygonData {
            polygon_stamped_msg: None,
            lines_in_static_frame: None,
            _tf_listener: tf_listener,
            _static_frame: static_frame,
            _color: config.color.to_tui(),
        }));

        let polygon = get_polygon_from_ros_param(&config.parameter);
        if let Some(polygon) = polygon {
            data.write().unwrap().polygon_stamped_msg =
                Some(rosrust_msg::geometry_msgs::PolygonStamped {
                    header: rosrust_msg::std_msgs::Header {
                        seq: 0,
                        stamp: rosrust::Time::new(),
                        frame_id: config.frame.clone(),
                    },
                    polygon: polygon,
                });
        }

        return ParameterPolygon { _data: data };
    }

    pub fn get_lines(&self) -> Vec<Line> {
        let mut unlocked_data = self._data.write().unwrap();
        unlocked_data.update();
        return unlocked_data.get_lines();
    }
}

impl PolygonListener {
    pub fn new(
        config: ListenerConfigColor,
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
    ) -> PolygonListener {
        let data = Arc::new(RwLock::new(PolygonData {
            polygon_stamped_msg: None,
            lines_in_static_frame: None,
            _tf_listener: tf_listener,
            _static_frame: static_frame,
            _color: config.color.to_tui(),
        }));

        let cloned_data = data.clone();
        let sub = rosrust::subscribe(
            &config.topic,
            1,
            move |msg: rosrust_msg::geometry_msgs::PolygonStamped| {
                let mut unlocked_data = cloned_data.write().unwrap();
                unlocked_data.polygon_stamped_msg = Some(msg);
                unlocked_data.update();
            },
        )
        .unwrap();

        return PolygonListener {
            _data: data,
            _subscriber: sub,
        };
    }

    pub fn get_lines(&self) -> Vec<Line> {
        return self._data.clone().read().unwrap().get_lines();
    }
}
