/*
Allocore Example: Flocking

Description:
This is an example implementation of a flocking algorithm. The original flocking
algorithm [1] consists of three main interactions between flockmates ("boids"):

    1) Collision avoidance (of nearby flockmates)
    2) Velocity matching (of nearby flockmates)
    3) Flock centering (of nearby flockmates)

Here, we implement 1) and 2) only. Another change from the reference source is
the use of Gaussian functions rather than inverse-squared functions for
calculating the "nearness" of flockmates. This is done primarily to avoid
infinities, but also to give smoother motions. Lastly, we give each boid a
random walk motion which helps both dissolve and redirect the flocks.

[1] Reynolds, C. W. (1987). Flocks, herds, and schools: A distributed behavioral
    model. Computer Graphics, 21(4):25–34.

Author:
Lance Putnam, Oct. 2014
*/

#include <cmath>
#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Functions.hpp"
#include "al/math/al_Random.hpp"
#include "al/app/al_GUIDomain.hpp"

using namespace al;

// A "boid" (play on bird) is one member of a flock.
class Boid {
 public:
  // Each boid has a position and velocity.
  Vec3f pos, vel;

  // Update position based on velocity and delta time
  void update(float dt) { pos += vel * dt; } //position-velocity gives last position
};

struct MyApp : public App {
  Nav agent;
  Nav target;
  double time{0};
  double angle{0};
  static const int Nb = 384;  // boids
  Boid boids[Nb];
  Mesh heads, tails;
  Mesh box;
  Mesh mesh;
  ParameterColor color{"Color"};
  //ParameterInt mode{"Mode", "", 1, 1, 4};
  Parameter maxAcceler{"/maxAcceler", "", 0.5, "", 0.01, 6.00};


  void onInit() override {
    // set up GUI
    auto GUIdomain = GUIDomain::enableGUI(defaultWindowDomain());
    auto& gui = GUIdomain->newGUI();
    //gui.add(mode);  // add parameter to GUI
    //gui.add(color);  // add parameter to GUI
    gui.add(maxAcceler);
    gui.add(color);
  }

  void onCreate() { //creates the box around the boids
    nav().pos(0.5, 0.7, 5);
    nav().faceToward(Vec3d(0, 0, 0));

    box.primitive(Mesh::LINE_LOOP); //try using lines instead of loops and put them in pairs of 2
    box.vertex(-5, -5, -5);
    box.vertex(5, -5, -5);
    box.vertex(5, 5, -5);
    box.vertex(-5, 5, -5);

    box.vertex(-5, 5, 5);
  //box.vertex(5, -5, 5);
  //box.vertex(5, 5, 5);
    box.vertex(-5, -5, 5);
    
    mesh.primitive(Mesh::LINES);
    mesh.vertex(0, 1, 0);
    mesh.color(0, 0, 0);
    mesh.vertex(0, 0, -2);
    mesh.color(1, 0, 0);
    mesh.vertex(-1, 0, 0);
    mesh.color(0, 1, 0);
    mesh.vertex(1, 0, 0);
    mesh.color(0, 0, 1);
    mesh.vertex(0, 1, 0);
    mesh.color(1, 0, 0);
        
    nav().pullBack(12);

    resetBoids();
  }

  // Randomize boid positions/velocities uniformly inside unit disc
  void resetBoids() {
    for (auto& b : boids) {
      b.pos = rnd::ball<Vec3f>();
      b.vel = rnd::ball<Vec3f>();
    }
  }

  void onAnimate(double dt_ms) {
    float dt = dt_ms;

    // Compute boid-boid interactions
    for (int i = 0; i < Nb - 1; ++i) {
      for (int j = i + 1; j < Nb; ++j) {
        // printf("checking boids %d and %d\n", i,j);

        auto ds = boids[i].pos - boids[j].pos;
        auto dist = ds.mag();

        // Collision avoidance
        float pushRadius = 0.1;
        float pushStrength = 1;
        float push = exp(-al::pow2(dist / pushRadius)) * pushStrength;

        auto pushVector = ds.normalized() * push;
    /*     boids[i].pos += pushVector;
        boids[j].pos -= pushVector; */

        boids[i].vel += pushVector;
        boids[j].vel -= pushVector;

        // Velocity matching
        float matchRadius = 0.125;
        float nearness = exp(-al::pow2(dist / matchRadius));
        Vec3f veli = boids[i].vel;
        Vec3f velj = boids[j].vel;

        // Take a weighted average of velocities according to nearness
        boids[i].vel = veli * (1 - 0.5 * nearness) + velj * (0.5 * nearness) * maxAcceler;
        boids[j].vel = velj * (1 - 0.5 * nearness) + veli * (0.5 * nearness);

        // TODO: Flock centering
      }
    }

    // Update boid independent behaviors
    for (auto& b : boids) {
      // Random "hunting" motion
      float huntUrge = 0.2;
      auto hunt = rnd::ball<Vec3f>();
      // Use cubed distribution to make small jumps more frequent
      hunt *= hunt.magSqr();
      b.vel += hunt * huntUrge;

      // Bound boid into a box
      if (b.pos.x > 5 || b.pos.x < -5) {
        b.pos.x = b.pos.x > 0 ? 5 : -5;
        b.vel.x = -b.vel.x;
      }
      if (b.pos.y > 5 || b.pos.y < -5) {
        b.pos.y = b.pos.y > 0 ? 5 : -5;
        b.vel.y = -b.vel.y;
      }
      if (b.pos.z > 1 || b.pos.z < -5) {
        b.pos.z = b.pos.z > 0 ? 5 : -5;
        b.vel.z = -b.vel.z;
      }
    }

    // Generate meshes
    heads.reset(); //resets the mess (colors etc)
    heads.primitive(Mesh::LINES);

    tails.reset(); // leaves trails if you comment them
    tails.primitive(Mesh::LINES); //comment these and let the magic happen

    for (int i = 1; i < Nb; ++i) { //exclude first boid
      boids[i].update(dt);

      heads.vertex(boids[i].pos);
      //heads.vertex(boids[i].pos + 0.1);
      heads.color(HSV(float(i) / Nb * 0.3 + 0.3, 0.7));

      tails.vertex(boids[i].pos);
      tails.vertex(boids[i].pos - boids[i].vel.normalized(0.07)); //makes the lines about the same length, otherwise it would show the size of velocity

      tails.color(heads.colors()[i]);
      tails.color(RGB(0.5));
    }
  }

  void onDraw(Graphics& g) {
    g.clear(color.get()); //changes background color. GUI color
    g.depthTesting(true);
    g.pointSize(8); //sets the pointSize of all points
    //g.nicest();
    //g.stroke(8);
    g.meshColor();
    g.draw(tails);

    // g.stroke(1);
    g.color();
    g.draw(box);

    for (auto &a : boids) {
      g.pushMatrix(); // push()
      g.translate(a.pos);
      //g.rotate(a.quat()); // rotate using the quat
      g.scale(0.01); //boid size
      g.draw(heads);
      g.popMatrix(); // pop()
    }
  }

  bool onKeyDown(const Keyboard& k) {
    switch (k.key()) {
      case 'r':
        resetBoids();
        break;
    }
    return true;
  }
};

int main() { MyApp().start(); }
