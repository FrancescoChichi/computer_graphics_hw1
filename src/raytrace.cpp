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

      sid = ybvh::add_point_shape(bvh_scn,
                                  (int) shape->points.size(), shape->points.data(),
                                  (int) shape->pos.size(), shape->pos.data(),
                                  shape->radius.data());
      ybvh::add_instance(bvh_scn,inst->xform(),
                         ym::inverse(inst->xform()), sid);

      sid = ybvh::add_line_shape(bvh_scn,
                                 (int) shape->lines.size(), shape->lines.data(),
                                 (int) shape->pos.size(), shape->pos.data(),
                                 shape->radius.data());
      ybvh::add_instance(bvh_scn,inst->xform(),
                         ym::inverse(inst->xform()), sid);

      sid = ybvh::add_triangle_shape(bvh_scn,
                                     (int) shape->triangles.size(), shape->triangles.data(),
                                     (int) shape->pos.size(), shape->pos.data(),
                                     shape->radius.data());
      ybvh::add_instance(bvh_scn,inst->xform(),
                         ym::inverse(inst->xform()), sid);

      sid = ybvh::add_tetra_shape(bvh_scn,
                                  (int) shape->tetras.size(), shape->tetras.data(),
                                  (int) shape->pos.size(), shape->pos.data(),
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
    auto k = scn->instances[intersection.iid]->msh->shapes[0]->mat->ks;
    v = {255,.0,255};
    //v = {k.x,k.y,k.z};
  }

  return v;

}

ym::image4f raytrace(const yobj::scene* scn, const ybvh::scene* bvh,
                     const ym::vec3f& amb, int resolution, int samples) {

  auto px = ym::vec<float, 4>(255.0);


  //scn->cameras[0]->rotation.x+=0.25;
  //scn->cameras[0]->translation.z+=6.8;
  //scn->cameras[0]->translation.y+=1.2;


  //yu::logging::log_info("Z: " + to_string(scn->cameras[0]->rotation.x) );

  auto cam = scn->cameras[0];
  int h = resolution;
  int w = (int) round(h * cam->aspect);

  ym::image4f img = ym::image4f(w,h, px);

  //cerr<<"w "<<w<<" h "<<h<<" yfov "<<cam->yfov<<" aspect "<<cam->aspect<<" res " <<resolution<<endl;


  /// antialiased with n^2 samplers per pixel
  for(int j = 0; j<h; j++) {
    for(int i = 0; i<w; i++) {
      img[{i,j}]=px;
      //for (int sj = 0; sj < samples; ++sj){
        //for (int si = 0; si < samples; ++si){
          //auto u = (i + (si+0.5f)/samples) / resolution;
          //auto v = (j + (sj+0.5f)/samples) / resolution;
          //auto ray = camera_ray(cam, u, v, w, h);
      auto u = (i +0.5f) / w;
      auto v = (j +0.5f) / h;
      auto ray = camera_ray(cam, u, v, w, h);
      //cerr<<"ray min"<<ray.tmin<< " max "<<ray.tmax<<endl;

      //std::cerr<<"img prima px "<<i<<" "<<j<<" x "<<img[{i,j}].x<<" y "<<img[{i,j}].y<<" z "<<img[{i,j}].z<<" w "<<img[{i,j}].w<<std::endl;

      //img[{i,j}].xyz() = compute_color(bvh, scn, ray);

      img[{i,j}].xyz() = compute_color(bvh, scn, ray);
      /*img[{i,j}].xyz().data()=a.data();
      img[{i,j}].xyz().y=a.y;
      img[{i,j}].xyz().z=a.z;
*/
     // yu::logging::log_info("dsadassada: " + to_string(img[{i,j}].xyz().x)+ "  " + to_string(img[{i,j}].xyz().y)+ "  " + to_string(img[{i,j}].xyz().z));

   //   yu::logging::log_info("color: x " + to_string(s.x)+" y: " + to_string(s.y)+" z: " + to_string(s.z));


      //std::cerr<<"img px "<<i<<" "<<j<<" x "<<img[{i,j}].x<<" y "<<img[{i,j}].y<<" z "<<img[{i,j}].z<<" w "<<img[{i,j}].w<<std::endl;
      //img[{i,j}] += compute_color(bvh, scn, ray);
        //}
      //}
      //img[{i,j}] /= (float) samples*samples;
    }
  }

  yu::logging::log_info("image h: " + to_string(h)+" w: " + to_string(w) );

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
