#include <thread>
#include "yocto_bvh.h"
#include "yocto_img.h"
#include "yocto_math.h"
#include "yocto_obj.h"
#include "yocto_utils.h"
#include "printData.h"
#include "math.h"

using namespace std;
vector<yobj::shape*> lights= vector<yobj::shape*>{};
float epsilon = 1e-4;

ym::vec3f triangleNormal(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto t = msh->shapes[0]->triangles[eid];
  return ym::normalize(euv.x * msh->shapes[0]->norm[t.x] +
                   euv.y * msh->shapes[0]->norm[t.y] + euv.z * msh->shapes[0]->norm[t.z]);
}
ym::vec3f lineNormal(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto t = msh->shapes[0]->lines[eid];
  return ym::normalize(euv.x * msh->shapes[0]->norm[t.x] +
                       euv.y * msh->shapes[0]->norm[t.y]);
}
ym::vec3f trianglePosition(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto l = msh->shapes[0]->triangles[eid];
  return (euv.x * msh->shapes[0]->pos[l.x] + euv.y * msh->shapes[0]->pos[l.y] +
         euv.z * msh->shapes[0]->pos[l.z]);
}
ym::vec3f linePosition(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto l = msh->shapes[0]->lines[eid];
  return (euv.x * msh->shapes[0]->pos[l.x] + euv.y * msh->shapes[0]->pos[l.y]);
}

ym::image4f flipImage4(ym::image4f img){
  auto W = img.width();
  auto H = img.height();
  ym::image4f m = ym::image4f(W,H, {0,0,0,1});
  for (int i = 0; i <W; ++i) {
    for (int j = 0; j <H; ++j) {
      m[{i,(H-1-j)}]=img[{i,j}];
    }
  }
  return m;
};

ym::vec4f textureLDR(const yobj::mesh* msh, int eid, ym::vec3f euv, yobj::texture* tex ){
  auto tr = msh->shapes[0]->triangles[eid];
  float u = euv.x*msh->shapes[0]->pos[tr.x].x + euv.y*msh->shapes[0]->pos[tr.y].x + euv.z*msh->shapes[0]->pos[tr.z].x;
  float v = euv.x*msh->shapes[0]->pos[tr.x].y + euv.y*msh->shapes[0]->pos[tr.y].y + euv.z*msh->shapes[0]->pos[tr.z].y;

  float s = (fmod(u,1.0f))*tex->width();
  float t = (fmod(v,1.0f))*tex->height();

  auto i = (int)round(s);
  auto j = (int)round(t);

  auto i1 = (int)round((i + 1)%tex->width());
  auto j1 = (int)round((j + 1)%tex->height());

  float wi = s - i;
  float wj = t - j;


  return ym::vec4f((1-wi)*(1-wj))*ym::srgb_to_linear(tex->ldr[{i,j}])
         +ym::vec4f(wi*(1-wj))*ym::srgb_to_linear(tex->ldr[{i1,j}])
         +ym::vec4f(wj*(1-wi))*ym::srgb_to_linear(tex->ldr[{i,j1}])
         +ym::vec4f(wi*wj)*ym::srgb_to_linear(tex->ldr[{i1,j1}]);
}
ym::vec4f textureHDR(const yobj::mesh* msh, int eid, ym::vec3f euv, yobj::texture* tex ){
  auto tr = msh->shapes[0]->triangles[eid];
  auto u = euv.x*msh->shapes[0]->pos[tr.x].x + euv.y*msh->shapes[0]->pos[tr.y].x + euv.z*msh->shapes[0]->pos[tr.z].x;
  auto v = euv.x*msh->shapes[0]->pos[tr.x].y + euv.y*msh->shapes[0]->pos[tr.y].y + euv.z*msh->shapes[0]->pos[tr.z].y;

  float s = (fmod(u,1.0f))*tex->width();
  float t = (fmod(v,1.0f))*tex->height();

  int i = (int)round(s);
  int j = (int)round(t);

  auto i1 = (i + 1)%tex->width();
  auto j1 = (j + 1)%tex->height();

  float wi = s - i;
  float wj = t - j;

  return ym::vec4f((1-wi)*(1-wj))*tex->hdr[{i,j}]
         +ym::vec4f(wi*(1-wj))*tex->hdr[{i1,j}]
         +ym::vec4f(wj*(1-wi))*tex->hdr[{i,j1}]
         +ym::vec4f(wi*wj)*tex->hdr[{i1,j1}];
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

    //printFrame(ym::to_frame(ist->xform()));
    //auto new_frame = ym::transform_frame(ym::inverse(ym::to_frame(scn->cameras[0]->xform())),ym::to_frame(ist->xform()));
    //auto iid =
    ybvh::add_instance(bvh_scn, ym::to_frame(ist->xform()),
                         shape_map.at(shp));
    //ybvh::set_instance_frame(bvh_scn,iid,new_frame);
  }

  ybvh::build_scene_bvh(bvh_scn);
  return bvh_scn;
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

  //ym::transform_ray(ym::inverse(ym::to_frame(scn->cameras[0]->xform())),ray);
  //ray=ym::transform_ray(ym::to_frame(scn->cameras[0]->xform()),ray);
  auto intersection = ybvh::intersect_scene(bvh, ray, false);
  for(auto ist:scn->instances){
    ray=ym::transform_ray(ym::to_frame(ist->xform()),ray);
    intersection = ybvh::intersect_scene(bvh, ray, false);
    if(intersection)
      break;
  }


//  ym::ray3f r = ym::ray3f(ym::transform_ray(ym::to_frame(scn->instances[intersection.iid]->xform()),ray));

  ym::vec4f c = ym::vec4f(0,0,0,1);

  if(intersection){
      auto ist = scn->instances[intersection.iid];
      auto mat = ist->msh->shapes[0]->mat;
      auto msh = ist->msh;
      auto kd = mat->kd;
      auto ks = mat->ks;
    if(!scn->instances[intersection.iid]->msh->shapes[0]->triangles.empty()){//triangles

      auto n =  triangleNormal(msh,intersection.eid,intersection.euv.xyz());
      auto p = trianglePosition(msh,intersection.eid,intersection.euv.xyz());
      if(mat->kd_txt!= nullptr)
      {
        if(mat->kd_txt->ldr) {
          kd*=textureLDR(msh, intersection.eid, intersection.euv.xyz(), mat->kd_txt).xyz();
        }
        else
          kd*=textureHDR(msh,intersection.eid,intersection.euv.xyz(), mat->kd_txt).xyz();
      }
      if(mat->ks_txt!= nullptr)
      {
        if(mat->ks_txt->ldr)
          ks*=textureLDR(msh,intersection.eid,intersection.euv.xyz(), mat->ks_txt).xyz();
        else
          ks*=textureHDR(msh,intersection.eid,intersection.euv.xyz(), mat->ks_txt).xyz();
      }
      for(auto light:lights){

        auto l = ym::normalize(light->pos[0]-p);
        auto r = ym::length(light->pos[0]-p);

        ym::ray3f sr = ym::ray3f(p,l,epsilon,r);
        auto shadow = ybvh::intersect_scene(bvh, sr, false);
        auto In = light->mat->ke / (r * r);
        if(shadow) {
          /*auto smsh = scn->instances[shadow.iid]->msh;
          auto sn =  triangleNormal(smsh,shadow.eid,shadow.euv.xyz());
          auto sp = trianglePosition(smsh,shadow.eid,shadow.euv.xyz());
          auto sl = ym::normalize(light->pos[0]-sp);

          if(shadow.iid==intersection.iid)
            c.xyz() += kd * max(.0f, dot(sn, sl));
          //c.xyz() += kd * max(.0f, dot(sn, normalize(light->pos[0]-sp)));*/
          continue;
        }
        else {
          auto v = ym::normalize(ray.o-p);
          auto h = ym::normalize((v+l));
          auto ns = (mat->rs) ? 2 / (mat->rs * mat->rs) - 2 : 1e6f;

          //if(!shadow || shadow.iid==intersection.iid)
          c.xyz() += kd * In * max(.0f, dot(n, l))
                   + ks * In * pow(max(.0f,dot(n,h)),ns);
        }
      }

      if(mat->opacity)
        c.xyz() *= ym::vec3f(mat->opacity)
                   + ym::vec3f(1-mat->opacity)
                     * compute_color(bvh,scn,ym::ray3f(p,ray.d,epsilon)).xyz();


      if(mat->kr.x!=0 && mat->kr.y!=0 && mat->kr.z!=0)//reflection
      {
        ym::vec3f n =  triangleNormal(msh,intersection.eid,intersection.euv.xyz());
        auto ist = scn->instances[intersection.iid];
        auto mat = ist->msh->shapes[0]->mat;

        ym::ray3f reflectionRay = ym::ray3f();
        reflectionRay.o=intersection.euv.xyz();
        float a = ym::dot(n,-ray.d);
        a *= 2;
        n.x*=a;
        n.y*=a;
        n.z*=a;//2(n*v)
        reflectionRay.d= n+ray.d;
        //avoid hitting visible point
        reflectionRay.tmin=epsilon;//0.0005f+0.0002f;
        // accumulate the reflected light (recursive call) scaled by the material reflection
        c.xyz() += mat->kr*compute_color(bvh,scn, reflectionRay).xyz();
      }



    }
    else{//lines
      auto n =  lineNormal(msh,intersection.eid,intersection.euv.xyz());
      auto p = linePosition(msh,intersection.eid,intersection.euv.xyz());

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

        c.xyz() += kd * In * max(.0f, dot(n, l));
        }
      }
    }
  }
  //c+={0.5, 0.2, 0.2,1};
  return {c.x, c.y, c.z, 0};
}

ym::image4f raytrace(const yobj::scene* scn, const ybvh::scene* bvh,
                     const ym::vec4f& amb, int resolution, int samples) {
  auto cam = scn->cameras[0];

  float h = 2*tan(cam->yfov/2);
  float w = h*cam->aspect;

  int height = resolution;
  int width = (int)round(resolution*cam->aspect);

  float norm = samples*samples;
  ym::image4f img = ym::image4f(width,height, {0,0,0,1});

  /// antialiased with n^2 samplers per pixel
  for(int j = 0; j<height; ++j) {
    for(int i = 0; i<width; ++i) {
      for(int sj=0; sj<samples;++sj) {
        for (int si = 0; si < samples; ++si) {
          auto u = (i + (si+0.5f)/samples) / width;
          auto v = (j + (sj+0.5f)/samples) / height;
          auto ray = camera_ray(cam, u, v, w, h);
          img[{i, j}] +=  compute_color(bvh,scn,ray);
        }
      }
      img[{i,j}]+=amb ;
      img[{i, j}] /= {norm,norm,norm,1};
    }
  }

  return flipImage4(img);
}



ym::image4f raytrace_mt(const yobj::scene* scn, const ybvh::scene* bvh,
                        const ym::vec4f& amb, int resolution, int samples) {
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
//  yobj::add_default_camera(scn);

  // create bvh
  yu::logging::log_info("creating bvh");
  auto bvh = make_bvh(scn);
  yu::logging::log_info("bvh created");

  // raytrace
  yu::logging::log_info("tracing scene");
  auto hdr = (parallel)
             ? raytrace_mt(scn, bvh, {amb, amb, amb, 1}, resolution, samples)
             : raytrace(scn, bvh, {amb, amb, amb, 1}, resolution, samples);
  // tonemap and save
  yu::logging::log_info("saving image " + imageout);
  auto ldr = ym::tonemap_image(hdr, ym::tonemap_type::srgb, 0, 2.2);
  yimg::save_image4b(imageout, ldr);
}
