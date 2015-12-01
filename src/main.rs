extern crate kiss3d;
extern crate zoom;
extern crate num;
extern crate nalgebra as na;
extern crate rand;

use rand::{SeedableRng, Rng};
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use zoom::*;
use num::Zero;
use na::Vec3 as NaVec;

type Vec3 = Cartesian3<f64>;

trait LorentzPhysics: Particle<Vec3, f64> {
    fn quanta(&self) -> f64;
    fn inertia(&self) -> f64;
}

impl Quanta<f64> for LorentzPhysics {
    fn quanta(&self) -> f64 {
        self.quanta()
    }
}

impl PhysicsParticle<Vec3, f64> for LorentzPhysics {
    fn inertia(&self) -> f64 {
        self.inertia()
    }
}

trait GravityPhysics: Particle<Vec3, f64> {
    fn quanta(&self) -> f64;
    fn inertia(&self) -> f64;
}

impl Quanta<f64> for GravityPhysics {
    fn quanta(&self) -> f64 {
        self.quanta()
    }
}

impl PhysicsParticle<Vec3, f64> for GravityPhysics {
    fn inertia(&self) -> f64 {
        self.inertia()
    }
}

struct Ball {
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
}

fn to_navec(v: Vec3) -> NaVec<f32> {
    NaVec::new(v.x as f32, v.y as f32, v.z as f32)
}

impl Ball {
    fn new(position: Vec3, velocity: Vec3) -> Self {
        Ball{
            position: position,
            velocity: velocity,
            acceleration: Vec3::zero(),
        }
    }
}

impl GravityPhysics for Ball {
    fn quanta(&self) -> f64 {
        1.0
    }
    fn inertia(&self) -> f64 {
        100000.0
    }
}

impl LorentzPhysics for Ball {
    fn quanta(&self) -> f64 {
        50.0
    }
    fn inertia(&self) -> f64 {
        100000.0
    }
}

impl Particle<Vec3, f64> for Ball {
    fn position(&self) -> Vec3 {
        self.position
    }

    fn velocity(&self) -> Vec3 {
        self.velocity
    }

    fn accelerate(&mut self, vec: &Vec3) {
        self.acceleration = self.acceleration + *vec;
    }

    fn advance(&mut self) {
        self.velocity = self.velocity + self.acceleration;
        self.position = self.position + self.velocity;
        self.acceleration = Vec3::zero();
    }
}

fn main() {
    let mut window = Window::new("zoom test");
    struct SphereBall {
        scene_node: SceneNode,
        ball: Ball,
    }
    let mut rng = rand::Isaac64Rng::from_seed(&[1, 2, 3, 4]);
    let mut sballs = (0..500).map(|_| SphereBall{
        scene_node: window.add_sphere(0.2),
        ball: Ball::new(Vec3::new(rng.next_f64() - 0.5, rng.next_f64() - 0.5, rng.next_f64() - 0.5) * 10.0, Vec3::zero())
    }).map(|mut sball| {sball.scene_node.set_color(0.0, 0.0, 1.0); sball}).collect::<Vec<_>>();

    window.set_light(Light::StickToCamera);

    while window.render() {
        for i in 0..sballs.len() {
            for j in (i+1)..sballs.len() {
                unsafe {
                    GravityPhysics::gravitate_radius_squared::<GravityPhysics>(&mut (*sballs.as_mut_ptr().offset(i as isize)).ball,
                        &mut (*sballs.as_mut_ptr().offset(j as isize)).ball, 0.5);
                    LorentzPhysics::lorentz_radius_squared::<LorentzPhysics>(&mut (*sballs.as_mut_ptr().offset(i as isize)).ball,
                        &mut (*sballs.as_mut_ptr().offset(j as isize)).ball, 0.5);
                }
            }
        }
        for sball in sballs.iter_mut() {
            sball.ball.advance();
            sball.scene_node.set_local_translation(to_navec(sball.ball.position()));
        }
    }
}
