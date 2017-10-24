#include <thread>
#include "yocto_bvh.h"
#include "yocto_img.h"
#include "yocto_math.h"
#include "yocto_obj.h"
#include "yocto_utils.h"

ybvh::scene* make_bvh(yobj::scene* scn) {

  ybvh::scene* bvh_scn = ybvh::make_scene();

  ///add shape data and transforms and add shape instances
  for (auto& inst : scn->instances) {
    int sid = 0;
    for (auto& shape : inst->msh->shapes) {
      sid = ybvh::add_point_shape(bvh_scn,
                                  shape->points.size(), shape->points.data(),
                                  shape->pos.size(), shape->pos.data(),
                                  shape->radius.data());
      ybvh::add_instance(bvh_scn,inst->xform(),
                         ym::inverse(inst->xform()), sid);

      sid = ybvh::add_line_shape(bvh_scn,
                                 shape->lines.size(), shape->lines.data(),
                                 shape->pos.size(), shape->pos.data(),
                                 shape->radius.data());
      ybvh::add_instance(bvh_scn,inst->xform(),
                         ym::inverse(inst->xform()), sid);

      sid = ybvh::add_triangle_shape(bvh_scn,
                                     shape->triangles.size(), shape->triangles.data(),
                                     shape->pos.size(), shape->pos.data(),
                                     shape->radius.data());
      ybvh::add_instance(bvh_scn,inst->xform(),
                         ym::inverse(inst->xform()), sid);

      sid = ybvh::add_tetra_shape(bvh_scn,
                                  shape->tetras.size(), shape->tetras.data(),
                                  shape->pos.size(), shape->pos.data(),
                                  shape->radius.data());
      ybvh::add_instance(bvh_scn,inst->xform(),
                         ym::inverse(inst->xform()), sid);
    }
  }
  ybvh::build_scene_bvh(bvh_scn);
  return bvh_scn;
}


ym::ray3f camera_ray(yobj::camera* cam, float u, float v, float w, float h){
  auto camera_pos = ym::to_frame(cam->xform());
  auto q = camera_pos.o
   + ((u - .5f)*w*camera_pos.x)
   + ((v - .5f)*h*camera_pos.y)
   - (cam->focus*camera_pos.z);

  auto d = q-camera_pos.o;
  //std::cerr<<"d x "<<d.x<<" y "<<d.y<<" z "<<d.z<<std::endl<<std::endl;

  ym::ray3f ray = ym::ray3f(camera_pos.o,d/(ym::length(d)));
  ray.o = camera_pos.o;
  ray.d = ym::normalize(d);
  //ray.d=normalize(Q-originPos);

  //std::cerr<<"ray: x "<<ray.d.x<<" y "<<ray.d.y<<" z "<<ray.d.z<<std::endl;

  return ray;
}

ym::vec4f compute_color(const ybvh::scene* bvh, const yobj::scene* scn, ym::ray3f ray){
  //auto px = ym::vec<float,4>(1);

  auto intersaction = ybvh::intersect_scene(bvh, ray, true);

  std::cerr<<intersaction.dist<<std::endl;

  //std::cerr<< scn.sc->instances[intersaction.iid]->msh->shapes[intersaction.sid]->name<<std::endl;
 // std::cerr<< scn->instances[2]->msh->shapes[0]->color[0].y<<std::endl;
 /* img[i,j] = shade(shp, isec.uv);
}
}
vec3f shade(instance* ist, vec3f uv) {
  return ist->mat->kd;

*/
}

ym::image4f raytrace(const yobj::scene* scn, const ybvh::scene* bvh,
                     const ym::vec3f& amb, int resolution, int samples) {

  auto px = ym::vec<float, 4>(1);
  px.x=.5;
  px.y=.8;
  px.z=.0;
  ym::image4f img = ym::image4f(resolution,resolution, px);
  auto cam = scn->cameras[1];
  float w = resolution * cam->aspect;
  float h = 2 * std::tan(cam->yfov / 2);


  /// antialiased with n^2 samplers per pixel
  for(int j = 0; j<resolution; j++) {
    for(int i = 0; i<resolution; i++) {
      img[{i,j}]=px;
      for (int sj = 0; sj < samples; ++sj){
        for (int si = 0; si < samples; ++si){
          auto u = (i + (si+0.5f)/samples) / resolution;
          auto v = (j + (sj+0.5f)/samples) / resolution;
          auto ray = camera_ray(cam, u, v, w, h);
          img[{i,j}] += compute_color(bvh, scn, ray);
        }
      }
      img[{i,j}] /= (float) samples*samples;
    }
  }
  return {img};
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
  yobj::add_default_camera(scn);

  // create bvh
  yu::logging::log_info("creating bvh");
  auto bvh = make_bvh(scn);
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
