#include <thread>
#include "yocto_bvh.h"
#include "yocto_img.h"
#include "yocto_math.h"
#include "yocto_obj.h"
#include "yocto_utils.h"
using namespace std;

void printFrame(ym::frame<float,3> M){
  printf("%.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g;\n\n",
         M.x.x,M.y.x,M.z.x,
          M.x.y,M.y.y,M.z.y,
         M.x.z,M.y.z,M.z.z);
}

void printFrame(ym::mat4f M){
  printf("%.6g,%.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g,%.6g;\n %.6g,%.6g,%.6g,%.6g;\n\n",
         M.x.x,M.y.x,M.z.x,M.w.x,
         M.x.y,M.y.y,M.z.y,M.w.y,
         M.x.z,M.y.z,M.z.z,M.w.z,
         M.x.w,M.y.w,M.z.w,M.w.w);
}

ybvh::scene* make_bvh(yobj::scene* scn) {

  ybvh::scene* bvh_scn = ybvh::make_scene();
  auto shape_map = std::map<yobj::shape*, int>{{nullptr, -1}};

  for (auto mesh : scn->meshes) {

    auto shape=mesh->shapes[0];

    if (!shape->points.empty()) {
      shape_map[shape] = ybvh::add_point_shape(bvh_scn,
                                               (int) shape->points.size(), shape->points.data(),
                                               (int) shape->pos.size(), shape->pos.data(),
                                               shape->radius.data());
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
    //printFrame(ym::to_frame(ist->xform()));
    //printFrame(ist->xform());
    auto shp = ist->msh->shapes[0];
//    shp->pos;
//    ym::to_frame(ist->xform())
//    ist->translation=shp->pos[0];

    /// translation
    ym::vec3f t = shp->pos[0];
    /// rotation

    ym::quat4f rotation = {0,0,0,1};
    ///xform
    ym::mat4f shape_frame = ym::translation_mat4(t) * ym::rotation_mat4(rotation) *
        ym::scaling_mat4(ist->scale) * ist->matrix;

    printFrame(shape_frame);

    auto iid = ybvh::add_instance(bvh_scn, ym::to_frame(ist->xform()),
                         shape_map.at(shp));
    ybvh::set_instance_frame(bvh_scn,iid,ym::to_frame(shape_frame));
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
  ym::ray3f ray = ym::ray3f(camera_pos.o,ym::normalize(d));

  return ray;
}

ym::vec3f compute_color(const ybvh::scene* bvh, const yobj::scene* scn, ym::ray3f ray){

  auto intersection = ybvh::intersect_scene(bvh, ray, false);

  ym::vec3f v = ym::vec3f(0,0,0);


  if(intersection){
//    printFrame(ym::to_frame(scn->instances[intersection.iid]->xform()));

    //printf("distanza %f \n", intersection.dist);
//    if(intersection.iid)
//      cout<<"obj: "<<scn->instances[intersection.iid]->name<<endl;

    auto k = scn->instances[intersection.iid]->msh->shapes[0]->mat->kd;
    v = {k.x,k.y,k.z};
  }

  return v;

}

ym::image4f raytrace(const yobj::scene* scn, const ybvh::scene* bvh,
                     const ym::vec3f& amb, int resolution, int samples) {


  auto cam = scn->cameras[0];

  float h = 2*tan(cam->yfov/2);
  float w = h*cam->aspect;

  int height = resolution;
  int width = round(resolution*cam->aspect);

  ym::image4f img = ym::image4f(width,height, ym::vec<float, 4>(255.0));

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
//  scn->cameras[0]->translation.z+=1.8;
//  scn->cameras[0]->translation.y+=1.8;

//  scn->cameras[0]->rotation.y+=0.1;
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
