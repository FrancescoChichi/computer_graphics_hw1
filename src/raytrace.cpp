#include <thread>
#include "yocto_bvh.h"
#include "yocto_img.h"
#include "yocto_math.h"
#include "yocto_obj.h"
#include "yocto_utils.h"
using namespace std;

ybvh::scene* make_bvh(yobj::scene* scn) {

  ybvh::scene* bvh_scn = ybvh::make_scene();

  int sid = 0;
  ///add shape data and transforms and add shape instances

  for (auto& inst : scn->instances) {
    for (auto& shape : inst->msh->shapes) {
      //yu::logging::log_info("lines "+to_string(shape->lines.size()) + " points " +to_string(shape->points.size())
        //                    + " triangoli " +to_string(shape->triangles.size()) + " tetra " + to_string(shape->tetras.size()));

      if(!shape->points.empty()){
        sid = ybvh::add_point_shape(bvh_scn,
                                    (int) shape->points.size(), shape->points.data(),
                                    (int) shape->pos.size(), shape->pos.data(),
                                    shape->radius.data());
      }
      else if(!shape->lines.empty()) {
        sid = ybvh::add_line_shape(bvh_scn,
                                   (int) shape->lines.size(), shape->lines.data(),
                                   (int) shape->pos.size(), shape->pos.data(),
                                   shape->radius.data());
      }
      else if(!shape->triangles.empty()) {
        sid = ybvh::add_triangle_shape(bvh_scn,
                                       (int) shape->triangles.size(), shape->triangles.data(),
                                       (int) shape->pos.size(), shape->pos.data(),
                                       shape->radius.data());
      }

      ybvh::add_instance(bvh_scn,inst->xform(),
                         ym::inverse(inst->xform()), sid);

    }
  }
  //yu::logging::log_info("******************");
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
  ym::ray3f ray = ym::ray3f();
  ray.d=ym::normalize(d);
  //std::cerr<<"d "<<d.x<<" "<<d.y<<" "<<d.z<<std::endl;
  ray.o=camera_pos.o;
  return ray;
}

ym::vec3f compute_color(const ybvh::scene* bvh, const yobj::scene* scn, ym::ray3f ray){
  //auto px = ym::vec<float,4>(1);

  auto intersection = ybvh::intersect_scene(bvh, ray, false);

  ym::vec3f v = ym::vec3f(0,0,0);

  if(intersection){
    //yu::logging::log_info("obj: " + (scn->instances[intersection.iid]->name) );
    if(intersection.iid!=0)
      yu::logging::log_info("obj: " + (scn->instances[intersection.iid]->name) );

    auto k = scn->instances[intersection.iid]->msh->shapes[0]->mat->kd;
    //v = {255,0,255};
    v = {k.x,k.y,k.z};
  }

  return v;

}

ym::image4f raytrace(const yobj::scene* scn, const ybvh::scene* bvh,
                     const ym::vec3f& amb, int resolution, int samples) {

  auto px = ym::vec<float, 4>(255.0);


//  scn->cameras[0]->rotation.x+=0.25;
//  scn->cameras[0]->translation.z+=9.2;
  //scn->cameras[0]->translation.y+=1.2;


  //yu::logging::log_info("Z: " + to_string(scn->cameras[0]->rotation.x) );

  auto cam = scn->cameras[0];
  float h = 2*tan(cam->yfov/2);
  float w = h*cam->aspect;
  int height = resolution;
  int width = abs(resolution*cam->aspect);
  ym::image4f img = ym::image4f(width,height, px);

  /// antialiased with n^2 samplers per pixel
  for(int j = 0; j<height; j++) {
    for(int i = 0; i<width; i++) {
      auto u = (i +0.5f) / width;
      auto v = (j +0.5f) / height;
      auto ray = camera_ray(cam, u, v, w, h);
      img[{i,j}].xyz() = compute_color(bvh, scn, ray);
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

  printf("ciao \n");

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
