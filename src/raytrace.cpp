#include <thread>
#include "yocto_bvh.h"
#include "yocto_img.h"
#include "yocto_math.h"
#include "yocto_obj.h"
#include "yocto_utils.h"
#include "printData.h"

using namespace std;
vector<yobj::shape*> lights= vector<yobj::shape*>{};
float epsilon = 1e-3;

ym::vec3f normal(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto t = msh->shapes[0]->triangles[eid];
  return ym::normalize(euv.x * msh->shapes[0]->norm[t.x] +
                   euv.y * msh->shapes[0]->norm[t.y] + euv.z * msh->shapes[0]->norm[t.z]);
}
ym::vec3f position(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto t = msh->shapes[0]->triangles[eid];
  return (euv.x * msh->shapes[0]->pos[t.x] + euv.y * msh->shapes[0]->pos[t.y] +
         euv.z * msh->shapes[0]->pos[t.z]);
}

ybvh::scene* make_bvh(yobj::scene* scn) {

  ybvh::scene* bvh_scn = ybvh::make_scene();
  auto shape_map = std::map<yobj::shape*, int>{{nullptr, -1}};

  for (auto mesh : scn->meshes) {

    auto shape=mesh->shapes[0];

    if (!shape->points.empty()) {
      lights.push_back(shape);
      continue;

    } else if (!shape->lines.empty()) {
      shape_map[shape] = ybvh::add_line_shape(bvh_scn,
                                              (int) shape->lines.size(), shape->lines.data(),
                                              (int) shape->pos.size(), shape->pos.data(),
                                              shape->radius.data());

    } else if (!shape->triangles.empty()) {
      shape_map[shape] = ybvh::add_triangle_shape(bvh_scn,
                                                  (int) shape->triangles.size(), shape->triangles.data(),
                                                  (int) shape->pos.size(), shape->pos.data(),
                                                  nullptr);
    } else {
      assert(false);
    }
  }

  for (auto ist : scn->instances) {

    auto shp = ist->msh->shapes[0];

    if(!shp->points.empty())
      continue;

//    auto new_frame = ym::inverse(ym::to_frame(scn->cameras[0]->matrix))*ym::to_frame(ist->matrix);
    auto iid = ybvh::add_instance(bvh_scn, ym::to_frame(ist->xform()),
                         shape_map.at(shp));
//    ybvh::set_instance_frame(bvh_scn,iid,new_frame);
  }

  ybvh::build_scene_bvh(bvh_scn);
  return bvh_scn;
}

void ray_triangle_intersection(ym::vec3f v0,ym::vec3f v1,ym::vec3f v2, ym::ray3f r){
  //auto v2 = r.o - v0;
  auto e1 = v1 - v0;
  ym::vec3f e2 = v2 - v0;
  auto n = ym::dot(ym::cross(r.d,e2),e1);
}

ym::ray3f camera_ray(yobj::camera* cam, float u, float v, float w, float h){

  auto camera_pos = ym::to_frame(cam->xform());
  //auto camera_pos = ym::inverse(ym::to_frame(cam->xform()));

  auto q = camera_pos.o
   + ((u - .5f)*w*camera_pos.x)
   + ((v - .5f)*h*camera_pos.y)
   - (camera_pos.z);


  auto d = q-camera_pos.o;
  ym::ray3f ray = ym::ray3f(camera_pos.o,ym::normalize(d),epsilon);

  return ray;
}

ym::vec4f compute_color(const ybvh::scene* bvh, const yobj::scene* scn, ym::ray3f ray){

  auto intersection = ybvh::intersect_scene(bvh, ray, false);


//  ym::ray3f r = ym::ray3f(ym::transform_ray(ym::to_frame(scn->instances[intersection.iid]->xform()),ray));

  ym::vec4f c = ym::vec4f(0,0,0,1);

  if(intersection){

    auto ist = scn->instances[intersection.iid];
    auto mat = ist->msh->shapes[0]->mat;
    auto msh = ist->msh;

    auto n =  normal(msh,intersection.eid,intersection.euv.xyz());
    auto p = position(msh,intersection.eid,intersection.euv.xyz());

    for(auto light:lights){

      auto l = normalize(light->pos[0]-p);
      auto r = length(light->pos[0]-p);

      ym::ray3f sr = ym::ray3f{p,l,epsilon,r};
      auto shadow = ybvh::intersect_scene(bvh, sr, false);
      if(shadow) {
        continue;
      }
      else {
        auto In = light->mat->ke / (r * r);
        auto v = normalize(ray.o-p);
        auto h = ym::normalize((v+l));
        auto ns = (mat->rs) ? 2 / (mat->rs * mat->rs) - 2 : 1e6f;

        c.xyz() += mat->kd * In * max(.0f, fabs(dot(n, l)))
                 + mat->ks * In * pow(max(.0f,fabs(dot(n,h))),ns);
      }
    }
  }

  return {c.x, c.y, c.z, 0};
}

ym::image4f raytrace(const yobj::scene* scn, const ybvh::scene* bvh,
                     const ym::vec3f& amb, int resolution, int samples) {
  auto cam = scn->cameras[0];

  float h = 2*tan(cam->yfov/2);
  float w = h*cam->aspect;

  int height = resolution;
  int width = (int)round(resolution*cam->aspect);

  float norm = samples*samples;
  ym::image4f img = ym::image4f(width,height, {0,0,0,1});

  /// antialiased with n^2 samplers per pixel
  for(int j = 0; j<height; j++) {
    for(int i = 0; i<width; i++) {
      for(int sj=0; sj<samples;sj++) {
        for (int si = 0; si < samples; si++) {
          auto u = (i + (si+0.5f)/samples) / width;
          auto v = (j + (sj+0.5f)/samples) / height;
          auto ray = camera_ray(cam, u, v, w, h);
          img[{i, j}] += compute_color(bvh,scn,ray);
        }
      }
      img[{i, j}] /= {norm,norm,norm,1};
    }
  }

  return img;
}



ym::image4f raytrace_mt(const yobj::scene* scn, const ybvh::scene* bvh,
                        const ym::vec3f& amb, int resolution, int samples) {
  // YOUR CODE GOES HERE ----------------------
  return {};
}

int main(int argc, char** argv) {

  // command line parsing
  auto parser =
      yu::cmdline::make_parser(argc, argv, "raytrace", "raytrace scene");
  auto parallel =
      yu::cmdline::parse_flag(parser, "--parallel", "-p", "runs in parallel");
  auto resolution = yu::cmdline::parse_opti(
      parser, "--resolution", "-r", "vertical resolution", 720);
  auto samples = yu::cmdline::parse_opti(
      parser, "--samples", "-s", "per-pixel samples", 1);
  auto amb = yu::cmdline::parse_optf(
      parser, "--ambient", "-a", "ambient color", 0.1f);
  auto imageout = yu::cmdline::parse_opts(
      parser, "--output", "-o", "output image", "out.png");
  auto scenein = yu::cmdline::parse_args(
      parser, "scenein", "input scene", "scene.obj", true);
  yu::cmdline::check_parser(parser);

  // load scene
  yu::logging::log_info("loading scene " + scenein);
  auto scn = yobj::load_scene(scenein, true);

  // add missing data
  yobj::add_normals(scn);
  yobj::add_radius(scn, 0.001f);
  yobj::add_instances(scn);
//  scn->cameras.clear();
  yobj::add_default_camera(scn);

  // create bvh
  yu::logging::log_info("creating bvh");
  auto bvh = make_bvh(scn);
  yu::logging::log_info("bvh created");

  // raytrace
  yu::logging::log_info("tracing scene");
  auto hdr = (parallel)
             ? raytrace_mt(scn, bvh, {amb, amb, amb}, resolution, samples)
             : raytrace(scn, bvh, {amb, amb, amb}, resolution, samples);
  // tonemap and save
  yu::logging::log_info("saving image " + imageout);
  auto ldr = ym::tonemap_image(hdr, ym::tonemap_type::srgb, 0, 2.2);
  yimg::save_image4b(imageout, ldr);
}
