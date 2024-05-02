#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "collision/dune.h"
#include "collision/cylinder.h"
#include "collision/cube.h"

#include "CGL/vector2D.h"
#include "linkageUtils.h"

using namespace std;

Beest::~Beest() {
  pms.clear();
  ss.clear();
}

void Beest::addTube() {

}

void Beest::buildBeest() {
  std::vector<int> fixedPoints = { 1, 2, 8 };
  pms.reserve(Jansen::nPoints);
  ss.reserve((22 + 3) * Jansen::nLegs - 3);
  for (int i = 0; i < Jansen::nPoints; i++) {
    pms.push_back(PointMass(Vector3D(0), false));
    spheres.push_back(Sphere(Vector3D(0), 0.005, 0));
  }

  Matrix3f rot = Matrix3f::Identity();
  // flip the x axis and z axis
  rot(0, 0) = -1;
  rot(2, 2) = -1;


  legModel = Jansen(
    Eigen::Matrix<float, Jansen::nCoords, 1>( 0 ),
    Eigen::Matrix<float, Jansen::nCoords, 1>( 0 ),
    Eigen::Vector3f(0.0f, 1.0f, 0.0f),
    rot
  );

  std::vector<CGL::Vector3D> positions = legModel.positions;
  for (int i = 0; i < Jansen::nPoints; i++) {
    pms[i].position = scale * positions[i];
    spheres[i].pose(pms[i].position);
  }

  int pPerSet = 14;

  for (int l = 0; l < Jansen::nLegs - 1; l++) {
    int idx = l * pPerSet;
    for (int p1 : fixedPoints) {
      ss.push_back(Spring(&(pms[idx + p1]), &pms[idx + pPerSet + p1], STRUCTURAL));
    }
  }

  //for (int l = 0; l < numLegs; l++) {
  for (int i = 0; i < legModel.nLegs; i++) {
    for (int p = 0; p < 2; p++) {
      int idx = i * pPerSet;
      int sIdx = idx + 6 * p;
      ss.push_back(Spring(&(pms[idx + 0]), &pms[idx + 1], SHEARING));
      ss.push_back(Spring(&(pms[idx + 1]), &pms[sIdx + 3], SHEARING));
      ss.push_back(Spring(&(pms[idx + 1]), &pms[sIdx + 6], SHEARING));
      ss.push_back(Spring(&(pms[sIdx + 2]), &pms[sIdx + 3], SHEARING));
      ss.push_back(Spring(&(pms[sIdx + 2]), &pms[sIdx + 4], SHEARING));
      ss.push_back(Spring(&(pms[sIdx + 3]), &pms[sIdx + 4], SHEARING));
      ss.push_back(Spring(&(pms[sIdx + 4]), &pms[sIdx + 5], SHEARING));
      ss.push_back(Spring(&(pms[sIdx + 2]), &pms[sIdx + 6], SHEARING));
      ss.push_back(Spring(&(pms[sIdx + 5]), &pms[sIdx + 6], SHEARING));
      ss.push_back(Spring(&(pms[sIdx + 5]), &pms[sIdx + 7], SHEARING));
      ss.push_back(Spring(&(pms[sIdx + 6]), &pms[sIdx + 7], SHEARING));
    }
  }

    /*ss.push_back(Spring(&(pms[idx + 0]), &pms[idx + 1], SHEARING));
    ss.push_back(Spring(&(pms[idx + 1]), &pms[idx + 9], SHEARING));
    ss.push_back(Spring(&(pms[idx + 1]), &pms[idx + 12], SHEARING));
    ss.push_back(Spring(&(pms[idx + 8]), &pms[idx + 9], SHEARING));
    ss.push_back(Spring(&(pms[idx + 8]), &pms[idx + 10], SHEARING));
    ss.push_back(Spring(&(pms[idx + 9]), &pms[idx + 10], SHEARING));
    ss.push_back(Spring(&(pms[idx + 10]), &pms[idx + 11], SHEARING));
    ss.push_back(Spring(&(pms[idx + 8]), &pms[idx + 12], SHEARING));
    ss.push_back(Spring(&(pms[idx + 11]), &pms[idx + 12], SHEARING));
    ss.push_back(Spring(&(pms[idx + 11]), &pms[idx + 13], SHEARING));
    ss.push_back(Spring(&(pms[idx + 12]), &pms[idx + 13], SHEARING));*/
  //}
  cs.clear();
  for (auto spring : ss) {
    Vector3D a = spring.pm_a->position;
    Vector3D b = spring.pm_b->position;
    Vector3D center = (a + b) / 2;
    Vector3D axis = a - b;
    double dist = (a - b).norm() / 2;
    cs.push_back(Cylinder(center, axis, 0.003, dist, 0.5));
  }
}

void Beest::simulate(double frames_per_sec, double simulation_steps,
  vector<Vector3D> external_accelerations,
  vector<CollisionObject*>* collision_objects,
  float ks) {

  vector<Vector3D> pointForces(4);
  float q = legModel.q(0, 0);
  float f = 660;
  pointForces[0] -= f * convertVector(legModel.rotation * Eigen::Vector3f(-sinf(q), cosf(q), 0));
  pointForces[1] += f * (1.5 / 7.6) * convertVector(legModel.rotation.col(1));
  pointForces[2] -= f * (1.5 / 7.6) * convertVector(legModel.rotation.col(1));
  pointForces[3] += f * convertVector(legModel.rotation * Eigen::Vector3f(-sinf(q), cosf(q), 0));
 
  legModel.simulate(frames_per_sec, simulation_steps,
    external_accelerations, collision_objects,
    pointForces, { 0, 2, 8, 1 }
  );
  vector<Vector3D> positions = legModel.positions;
  for (int i = 0; i < Jansen::nPoints; i++) {
    pms[i].position = scale * positions[i];
    spheres[i].pose(pms[i].position);
  }
  for (int i = 0; i < ss.size(); i++) {
    Spring& spring = ss[i];
    Vector3D a = spring.pm_a->position;
    Vector3D b = spring.pm_b->position;
    Vector3D center = (a + b) / 2;
    Vector3D axis = a - b;
    cs[i].pose(center, axis);
  }
}

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}
void Cloth::buildGrid() {
    beest = Beest(0.1);
    beest.buildBeest();
    // TODO (Part 1): Build a grid of masses and springs.
    for (int i = 0; i < num_height_points; i++) {
        for (int j = 0; j < num_width_points; j++) {
            bool isPinned = false;
            for (int k = 0; k < pinned.size(); k++) {
                if (i == pinned[k][1] && j == pinned[k][0]) {
                    isPinned = true;
                    break;
                }
            }
            if (orientation == HORIZONTAL) {
                point_masses.emplace_back(PointMass(Vector3D(j * width / (num_width_points - 1), 1.0, i * height / (num_height_points - 1)), isPinned));
            }
            else {
                float random = (rand() * 1.0 / RAND_MAX - 0.1) * 0.001;
                point_masses.emplace_back(PointMass(Vector3D(j * width / (num_width_points - 1), i * height / (num_height_points - 1), random), isPinned));
            }
        }
    }

  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      // grab the current point mass
      PointMass *pm = &point_masses[i * num_width_points + j];
        
      //Structural constraints exist between a point mass and the point mass to its left as well as the point mass above it.
      if (j - 1 >= 0) {
        //to the left
        springs.push_back(Spring(pm, &point_masses[i * num_width_points + j - 1], STRUCTURAL));
      }
      if (i - 1 >= 0) {
        //above it
        springs.push_back(Spring(pm, &point_masses[(i - 1) * num_width_points + j], STRUCTURAL));
      }

      //Shearing constraints exist between a point mass and the point mass to its diagonal upper left as well as the point mass to its diagonal upper right.
      if (j + 1 < num_width_points && i - 1 >= 0) {
        //upper right
        springs.push_back(Spring(pm, &point_masses[(i - 1) * num_width_points + j + 1], SHEARING));
      }
      if (j - 1 >= 0 && i - 1 >= 0) {
        //upper left
        springs.push_back(Spring(pm, &point_masses[(i - 1) * num_width_points + j - 1], SHEARING));
      }

      //Bending constraints exist between a point mass and the point mass two away to its left as well as the point mass two above it.
      if (j - 2 >= 0) {
        //two to the left
        springs.push_back(Spring(pm, &point_masses[i * num_width_points + j - 2], BENDING));
      }
      if (i - 2 >= 0) {
        //two above it
        springs.push_back(Spring(pm, &point_masses[(i - 2) * num_width_points + j], BENDING));
      }
    }
  }
}


void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  // std::cout << beest.point_masses.size() << std::endl;
  beest.simulate(frames_per_sec, simulation_steps,
    external_accelerations, collision_objects, cp->ks);
  // for (int i = 0; i < beest.point_masses.size(); i++) {
  //   std::cout << beest.point_masses[i].position << std::endl;
  // }
  return;
  // double mass = width * height * cp->density / num_width_points / num_height_points;
  // double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // // TODO (Part 2): Compute total force acting on each point mass.
  // for (int i = 0; i < point_masses.size(); i++) {
	// point_masses[i].forces = Vector3D(0, 0, 0);
  //   for (int j = 0; j < external_accelerations.size(); j++) {
  //       point_masses[i].forces += mass * external_accelerations[j];
  //   }
  // }


  // // for each spring, add structural, shearing, and bending forces to the point masses if enabled
  // for (auto& spring : springs) {
  //     e_spring_type s_type = spring.spring_type;
  //     // if the type is enabled in the ClothParameters, add the spring force to the point masses
  //     bool enabled = false;
  //     if (s_type == STRUCTURAL) {
	// 	  enabled = cp->enable_structural_constraints;
	//   }
  //     else if (s_type == SHEARING) {
	// 	  enabled = cp->enable_shearing_constraints;
	//   }
  //     else if (s_type == BENDING) {
	// 	  enabled = cp->enable_bending_constraints;
	//   }
  //     if (enabled) {
	// 	  Vector3D delta21 = spring.pm_b->position - spring.pm_a->position;
	// 	  double dist = delta21.norm();
	// 	  double overage = dist - spring.rest_length;
  //         Vector3D force = cp->ks * overage * delta21.unit();
  //         if (s_type == BENDING) {
  //             force *= 0.02;
  //         }
  //         spring.pm_a->forces += force;
  //         spring.pm_b->forces -= force;
	//   }
  // }


  // // TODO (Part 2): Use Verlet integration to compute new point mass positions
  // for (int i = 0; i < point_masses.size(); i++) {
  //     if (!point_masses[i].pinned) {
	//   Vector3D temp = point_masses[i].position;
	//   point_masses[i].position = temp + (1 - cp->damping / 100) * (temp - point_masses[i].last_position) + (delta_t * delta_t * (point_masses[i].forces / mass));
	//   point_masses[i].last_position = temp;
	// }
  // }

  // // TODO (Part 4): Handle self-collisions.
  // // TODO (Part 4): Handle self-collisions.
  // build_spatial_map();
  // for (int i = 0; i < point_masses.size(); i++) {
  //   self_collide(point_masses[i], simulation_steps);
  // }


  // TODO (Part 3): Handle collisions with other primitives.
  for (PointMass& pm : point_masses) {
    if (!pm.pinned) {
      for (CollisionObject* co : *collision_objects) {
        co->collide(pm);
      }
    }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (auto& spring : springs) {
	  PointMass *pm1 = spring.pm_a;
    PointMass *pm2 = spring.pm_b;
    Vector3D delta21 = pm2->position - pm1->position;
    Vector3D delta12 = pm1->position - pm2->position;
    double dist = delta21.norm();
    double overage = dist - spring.rest_length * 1.1;
    if (overage > 0) {
        Vector3D correction1 = overage * delta21.unit();
        Vector3D correction2 = overage * delta12.unit();

    }
  }
  //       if (!pm1->pinned && !pm2->pinned) {
  //           pm1->position += correction1 / 2;
  //           pm2->position += correction2 / 2;
  //       }
  //       else if (!pm1->pinned) {
	// 		pm1->position += correction1;
  //       }
  //       else if (!pm2->pinned) {
	// 		pm2->position += correction2;
	// 	}
	// }
  // }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

  for (int i = 0; i < point_masses.size(); i++) {
    float hash = hash_position(point_masses[i].position);
    if (map.find(hash) == map.end()) {
      map[hash] = new vector<PointMass *>();
    }
    map[hash]->push_back(&point_masses[i]);
  }
}


void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

  float point_hash = hash_position(pm.position);

  vector<PointMass *> *point_masses = map[point_hash];
  Vector3D correction = Vector3D(0, 0, 0);
  int correction_counter = 0;

  for (int i = 0; i < point_masses->size(); i++) {
    PointMass *other_pm = (*point_masses)[i];
    Vector3D delta = pm.position - other_pm->position;
    float dist = delta.norm();
    float overage = 2 * thickness;
    if (other_pm != &pm && dist > 0 && dist < 2 * thickness) {
      delta.normalize();
      correction += (other_pm->position + delta * overage) - pm.position;
      correction_counter +=1;
    }
  }
  if (correction_counter > 0) {
    pm.position += correction / (correction_counter * simulation_steps);
}
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4.1): Hash a 3D position into a unique float identifier that represents
  // membership in some un  iquely identified 3D box volume.
  float w = 3 * width / num_width_points;
  float h = 3 * height / num_height_points;
  float t = max(w, h);

  if (orientation == HORIZONTAL) {
    return floor(pos.x / w) + floor(pos.y / h) + floor(pos.z / t);
  }
  if (orientation == VERTICAL) {
    return floor(pos.x / w) + floor(pos.y / t) + floor(pos.z / h);
  }
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
  beest.legModel.reset();
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
