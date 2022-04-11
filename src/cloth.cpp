#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  double dx = width / num_width_points;
  double dy = height / num_height_points;
  float x, y, z;


  for (int j = 0; j < num_height_points; j++) {
      for (int i = 0; i < num_width_points; i++) {
          bool pin = false;
          if (orientation == HORIZONTAL) {
              y = 1.0;
              x = i * dx;
              z = j * dy;
          } else if (orientation == VERTICAL) {
              z = rand() / RAND_MAX * (1/ 2000) - (1 / 1000);
              x = i * dx;
              y = j * dy;
          }
          for (int k = 0; k < pinned.size(); k++) {
              if (i == pinned[k][0] && j == pinned[k][1]) {
                  pin = true;
              }
          }
          PointMass pm = PointMass(Vector3D(x,y,z), pin);
          point_masses.push_back(pm);
      }
  }

  //springs
  for (int j = 0; j < num_height_points; j++) {
      for (int i = 0; i < num_width_points; i++) {
          int position = num_width_points * j + i;
          PointMass *pm = &point_masses[position];

          //Structural constraints exist between a point mass and the point mass to its left as well as the point mass above it.
          if (j >= 1) {
              //above
              int positionAbove = i + (j-1) * num_width_points;
              springs.emplace_back(Spring(pm, &point_masses[positionAbove], CGL::STRUCTURAL));
          }
          if (i >= 1) {
              // left
              int positionLeft = (i - 1) + j * num_width_points;
              springs.emplace_back(Spring(pm, &point_masses[positionLeft], CGL::STRUCTURAL));
          }
          //Shearing constraints exist between a point mass and the point mass to its diagonal upper left as well as the point mass to its diagonal upper right.
          if (i >= 1 && j >= 1) {
              //upper left
              int positionUpLeft = (i-1) + (j-1) * num_width_points;
              springs.emplace_back(Spring(pm, &point_masses[positionUpLeft], CGL::SHEARING));
          }
          if (i < num_width_points - 1 && j >= 1) {
              //upper right -- check not all the way to the right, and that upper is not 0
              int positionUpRight = (i+1) + (j-1) * num_width_points;
              springs.emplace_back(Spring(pm, &point_masses[positionUpRight], CGL::SHEARING));
          }
          //Bending constraints exist between a point mass and the point mass two away to its left as well as the point mass two above it.
          if (i >= 2) {
              //two to the left
              int positionTwoLeft = (i-2) + j * num_width_points;
              springs.emplace_back(Spring(pm, &point_masses[positionTwoLeft], CGL::BENDING));
          }
          if (j >=2) {
              int positionTwoAbove = i + (j-2) * num_width_points;
              springs.emplace_back(Spring(pm, &point_masses[positionTwoAbove], CGL::BENDING));
          }
      }
  }
//    for (PointMass& pm : this->point_masses) {
//        int x = pm.position.x;
//        int y = pm.position.y;
//            int position = num_width_points * y + x;
//
//            //Structural constraints exist between a point mass and the point mass to its left as well as the point mass above it.
//            if (y >= 1) {
//                //above
//                int positionAbove = x + (y-1) * num_width_points;
//                springs.emplace_back(Spring(&pm, &point_masses[positionAbove], CGL::STRUCTURAL));
//            }
//            if (x >= 1) {
//                // left
//                int positionLeft = (x - 1) + y * num_width_points;
//                springs.emplace_back(Spring(&pm, &point_masses[positionLeft], CGL::STRUCTURAL));
//            }
//            //Shearing constraints exist between a point mass and the point mass to its diagonal upper left as well as the point mass to its diagonal upper right.
//            if (x >= 1 && y >= 1) {
//                //upper left
//                int positionUpLeft = (x-1) + (y-1) * num_width_points;
//                springs.emplace_back(Spring(&pm, &point_masses[positionUpLeft], CGL::SHEARING));
//            }
//            if (x < num_width_points - 1 && y >= 1) {
//                //upper right -- check not all the way to the right, and that upper is not 0
//                int positionUpRight = (x+1) + (y-1) * num_width_points;
//                springs.emplace_back(Spring(&pm, &point_masses[positionUpRight], CGL::SHEARING));
//            }
//            //Bending constraints exist between a point mass and the point mass two away to its left as well as the point mass two above it.
//            if (x >= 2) {
//                //two to the left
//                int positionTwoLeft = (x-2) + y * num_width_points;
//                springs.emplace_back(Spring(&pm, &point_masses[positionTwoLeft], CGL::BENDING));
//            }
//            if (x >=2) {
//                int positionTwoAbove = x + (y-2) * num_width_points;
//                springs.emplace_back(Spring(&pm, &point_masses[positionTwoAbove], CGL::BENDING));
//            }
//    }

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.

  for (int j = 0; j < num_height_points; j++) {
      for (int i = 0; i < num_width_points; i++) {
          int index = i + j * num_width_points;
          point_masses[index].forces = 0;
      }
  }

  for (PointMass& pm: point_masses) {
      pm.forces += external_accelerations[0] * mass;
  }

  for (Spring spring: springs) {
      Vector3D pA = spring.pm_a->position;
      Vector3D pB = spring.pm_b->position;
      double k = cp->ks * 0.2;
      Vector3D totalForce;

      if (cp->enable_bending_constraints) {
          totalForce = k * ((pA-pB).norm() - spring.rest_length) * (pB - pA).unit();
          spring.pm_a->forces += totalForce;
          spring.pm_b->forces -= totalForce;
      }
      if (cp->enable_shearing_constraints) {
          totalForce = cp->ks * ((pA-pB).norm() - spring.rest_length) * (pB - pA).unit();
          spring.pm_a->forces += totalForce;
          spring.pm_b->forces -= totalForce;
      }
      if (cp->enable_structural_constraints) {
          totalForce = cp->ks * ((pA-pB).norm() - spring.rest_length) * (pB - pA).unit();
          spring.pm_a->forces += totalForce;
          spring.pm_b->forces -= totalForce;
      }
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (int j = 0; j < num_height_points; j++) {
      for (int i = 0; i< num_width_points; i++) {
          int index = i + j * num_width_points;
          if (!point_masses[index].pinned) {
              double d = cp->damping / 100;
              Vector3D at = point_masses[index].forces / mass;
              Vector3D curr = point_masses[index].position;
              Vector3D last = point_masses[index].last_position;
              Vector3D newPos = curr + (1-d) * (curr - last) + (at * delta_t * delta_t);

              point_masses[index].last_position = curr;
              point_masses[index].position = newPos;
          }
      }
  }

  // TODO (Part 4): Handle self-collisions.


  // TODO (Part 3): Handle collisions with other primitives.
    for (int i = 0; i < (*collision_objects).size(); i++) {
        CollisionObject* c = (*collision_objects)[i];
        for (int j = 0; j < point_masses.size(); j++) {
            PointMass* pm = &point_masses[j];
            c->collide(*pm);
        }
    }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
//  for (int j = 0; j < num_height_points; j++) {
//      for (int i = 0; i < num_width_points; i++) {
    for (Spring spring: springs) {
          PointMass *pA = spring.pm_a;
          PointMass *pB = spring.pm_b;
          Vector3D pA_pos = pA->position;
          Vector3D pB_pos = pB->position;

          double length = (pA_pos - pB_pos).norm();
          double maxLength = spring.rest_length * 1.1;

          bool tooLong = length - maxLength > 0;
          if (tooLong) {
              //check all pin cases
              if (!pA->pinned && !pB->pinned) {
                  //both are not pinned
                  pA_pos += (pB_pos - pA_pos).unit() * 0.5 * (length - maxLength);
                  pB_pos += (pB_pos - pA_pos).unit() * 0.5 * (length - maxLength);
              } else if (pA->pinned){
                  //just A is pinned, change B
                  pB_pos += (pB_pos - pA_pos).unit() * (length - maxLength);
              } else if (pB->pinned) {
                  //B is pinned, change A
                  pA_pos += (pB_pos - pA_pos).unit() * (length - maxLength);
              }
          }
      }
  //}

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
    PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
      float hash = hash_position(pm->position);
      if (map.find(hash) != map.end()) {
          map[hash]->push_back(pm);
      } else {
          map[hash] = new vector<PointMass*>;
      }
      pm++;
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  int w = 3 * width / num_width_points;
  int h = 3 * height / num_height_points;
  int t = max(w,h);

  int x = floor(pos.x / w);
  int y = floor(pos.y / h);
  int z = floor(pos.z / t);

  return x + pow(y,2) + pow(z, 3);
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
