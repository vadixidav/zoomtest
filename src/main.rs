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

//Threading imports
use std::sync::mpsc;
use std::thread;

type Vec3 = Cartesian3<f64>;

trait LorentzPhysics: Particle<Vec3, f64> + Inertia<f64> {
    fn quanta(&self) -> f64;
}

impl Quanta<f64> for LorentzPhysics {
    fn quanta(&self) -> f64 {
        self.quanta()
    }
}

impl PhysicsParticle<Vec3, f64> for LorentzPhysics {}

trait SpringPhysics: Particle<Vec3, f64> + Inertia<f64> {
    fn quanta(&self) -> f64;
}

impl Quanta<f64> for SpringPhysics {
    fn quanta(&self) -> f64 {
        self.quanta()
    }
}

impl PhysicsParticle<Vec3, f64> for SpringPhysics {}

trait GravityPhysics: Particle<Vec3, f64> + Inertia<f64> {
    fn quanta(&self) -> f64;
}

impl Quanta<f64> for GravityPhysics {
    fn quanta(&self) -> f64 {
        self.quanta()
    }
}

impl PhysicsParticle<Vec3, f64> for GravityPhysics {}

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

impl UniformBall<f64> for GravityPhysics {
    fn radius(&self) -> f64 {
        0.1
    }

    fn radius_squared(&self) -> f64 {
        0.001
    }
}

impl UniformBall<f64> for LorentzPhysics {
    fn radius(&self) -> f64 {
        0.1
    }

    fn radius_squared(&self) -> f64 {
        0.001
    }
}

impl Position<Vec3> for Ball {
    fn position(&self) -> Vec3 {
        self.position
    }
}

impl Velocity<Vec3> for Ball {
    fn velocity(&self) -> Vec3 {
        self.velocity
    }
}

impl Particle<Vec3, f64> for Ball {
    fn accelerate(&mut self, vec: &Vec3) {
        self.acceleration = self.acceleration + *vec;
    }

    fn advance(&mut self, time: f64) {
        self.velocity = self.velocity + self.acceleration * time;
        self.position = self.position + self.velocity * time;
        self.acceleration = Vec3::zero();
    }
}

impl Inertia<f64> for Ball {
    fn inertia(&self) -> f64 {
        100000.0
    }
}

impl GravityPhysics for Ball {
    fn quanta(&self) -> f64 {
        10.0
    }
}

impl LorentzPhysics for Ball {
    fn quanta(&self) -> f64 {
        200.0
    }
}

impl SpringPhysics for Ball {
    fn quanta(&self) -> f64 {
        20.0
    }
}

fn main() {
    let mut window = Window::new("zoom test");
    struct SphereBall {
        scene_node: SceneNode,
        ball: Ball,
    }
    #[derive(Clone)]
    struct SBPointer {
        pointer: *mut SphereBall,
    }
    unsafe impl Sync for SphereBall {}
    unsafe impl Send for SphereBall {}
    unsafe impl Send for SBPointer {}
    impl Copy for SBPointer {}
    let mut rng = rand::Isaac64Rng::from_seed(&[1, 3, 3, 4]);
    let mut sballs = (0..1200).map(|_| SphereBall{
        scene_node: window.add_sphere(0.2)/*window.add_cube(0.2, 0.2, 0.2)*/,
        ball: Ball::new(Vec3::new(rng.next_f64() - 0.5, rng.next_f64() - 0.5, rng.next_f64() - 0.5) * 10.0,
            Vec3::zero())
    }).map(|mut sball| {sball.scene_node.set_color(0.0, 0.0, 1.0); sball}).collect::<Vec<_>>();

    window.set_light(Light::StickToCamera);

    let threadTotal = 4;

    while window.render() {
        let (tx, rx) = mpsc::channel();
        let len = sballs.len();
        let sballptr = SBPointer{pointer: sballs.as_mut_ptr()};
        for t in 0..threadTotal {
            let tx = tx.clone();
            thread::spawn(move || {
                for i in (len*t/threadTotal)..(len*(t+1)/threadTotal) {
                    unsafe {
                        SpringPhysics::hooke_to::<SpringPhysics>(&mut (*sballptr.pointer.offset(i as isize)).ball,
                            &(*sballptr.pointer.offset(((i - 1 + len) % len) as isize)).ball, 7.0);
                        SpringPhysics::hooke_to::<SpringPhysics>(&mut (*sballptr.pointer.offset(i as isize)).ball,
                            &(*sballptr.pointer.offset(((i + 1 + len) % len) as isize)).ball, 7.0);
                    }

                    for j in 0..len {
                        if i != j {
                            unsafe {
                                GravityPhysics::gravitate_radius_to::<GravityPhysics>(&mut (*sballptr.pointer.offset(i as isize)).ball,
                                    &(*sballptr.pointer.offset(j as isize)).ball, -10.0);
                                LorentzPhysics::lorentz_radius_to::<LorentzPhysics>(&mut (*sballptr.pointer.offset(i as isize)).ball,
                                    &(*sballptr.pointer.offset(j as isize)).ball, 0.8);
                            }
                        }
                    }
                    tx.send(());
                }
            });
        }
        for _ in 0..threadTotal {
            let _ = rx.recv();
        }
        for sball in sballs.iter_mut() {
            GravityPhysics::drag(&mut sball.ball, 1500.0);
            sball.ball.advance(0.05);
            sball.scene_node.set_local_translation(to_navec(sball.ball.position()));
        }
    }
}
