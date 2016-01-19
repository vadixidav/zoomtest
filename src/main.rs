extern crate kiss3d;
extern crate zoom;
extern crate num;
extern crate nalgebra as na;
extern crate rand;
extern crate crossbeam;

use rand::{SeedableRng, Rng};
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use zoom::*;
use num::{Zero, Float};
use na::Vec3 as NaVec;

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

struct Thing {
    particle: BasicParticle<Vec3, f64>,
}

fn to_navec(v: Vec3) -> NaVec<f32> {
    NaVec::new(v.x as f32, v.y as f32, v.z as f32)
}

impl Thing {
    fn new(position: Vec3, velocity: Vec3) -> Self {
        Thing{
            particle: BasicParticle::new(1.0, position, velocity, 100000.0)
        }
    }
}

impl Ball<f64> for GravityPhysics {
    fn radius(&self) -> f64 {
        0.1
    }
}

impl Ball<f64> for LorentzPhysics {
    fn radius(&self) -> f64 {
        0.1
    }
}

impl Position<Vec3> for Thing {
    fn position(&self) -> Vec3 {
        self.particle.position()
    }
}

impl Velocity<Vec3> for Thing {
    fn velocity(&self) -> Vec3 {
        self.particle.velocity()
    }
}

impl Particle<Vec3, f64> for Thing {
    fn impulse(&self, vec: &Vec3) {
        self.particle.impulse(vec);
    }

    fn advance(&mut self, time: f64) {
        self.particle.advance(time);
    }
}

impl Inertia<f64> for Thing {
    fn inertia(&self) -> f64 {
        self.particle.inertia()
    }
}

impl GravityPhysics for Thing {
    fn quanta(&self) -> f64 {
        10.0
    }
}

impl LorentzPhysics for Thing {
    fn quanta(&self) -> f64 {
        200.0
    }
}

impl SpringPhysics for Thing {
    fn quanta(&self) -> f64 {
        20.0
    }
}

fn main() {
    let mut window = Window::new("zoom test");
    struct SphereBall {
        scene_node: SceneNode,
        ball: Thing,
    }
    unsafe impl Sync for SphereBall {}
    unsafe impl Send for SphereBall {}
    let mut rng = rand::Isaac64Rng::from_seed(&[1, 3, 3, 4]);
    let mut sballs = (0..1000).map(|_| SphereBall{
        scene_node: window.add_sphere(0.2)/*window.add_cube(0.2, 0.2, 0.2)*/,
        ball: Thing::new(Vec3::new(rng.next_f64() - 0.5, rng.next_f64() - 0.5, rng.next_f64() - 0.5) * 10.0,
            Vec3::zero())
    }).enumerate().map(|(i, mut sball)| {sball.scene_node.set_color((i as f32 * 0.134).sin()*0.8 + 0.2, (i as f32 * 0.17).sin()*0.8 + 0.2, (i as f32 * 0.2).sin()*0.8 + 0.2); sball}).collect::<Vec<_>>();

    window.set_light(Light::StickToCamera);

    let thread_total = 4;

    while window.render() {
        crossbeam::scope(|scope| {
            let len = sballs.len();
            let sballs = &sballs;
            let handles = (0..thread_total).map(|t| {
                scope.spawn(move || {
                    for i in (len*t/thread_total)..(len*(t+1)/thread_total) {
                        SpringPhysics::hooke_to::<SpringPhysics>(&sballs[i].ball,
                            &sballs[((i + len - 1) % len)].ball, 2.0);
                        SpringPhysics::hooke_to::<SpringPhysics>(&sballs[i].ball,
                            &sballs[((i + len + 1) % len)].ball, 2.0);
                        SpringPhysics::hooke_to::<SpringPhysics>(&sballs[i].ball,
                            &sballs[((i + len + len/7) % len)].ball, 2.0);

                        for j in 0..len {
                            if i != j {
                                GravityPhysics::gravitate_radius_to::<GravityPhysics>(&sballs[i].ball,
                                    &sballs[j].ball, -10.0);
                                LorentzPhysics::lorentz_radius_to::<LorentzPhysics>(&sballs[i].ball,
                                    &sballs[j].ball, 0.05);
                            }
                        }
                    }
                })
            }).collect::<Vec<_>>();

            for handle in handles {
                handle.join();
            }
        });
        for sball in sballs.iter_mut() {
            GravityPhysics::drag(&mut sball.ball, 30000.0);
            sball.ball.advance(0.5);
            sball.scene_node.set_local_translation(to_navec(sball.ball.position()));
        }
    }
}
