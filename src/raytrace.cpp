#include <thread>
#include "yocto_bvh.h"
#include "yocto_img.h"
#include "yocto_math.h"
#include "yocto_obj.h"
#include "yocto_utils.h"
#include "printData.h"
#include "math.h"

vector<yobj::instance*> lights;
float epsilon = 1e-4;

ym::vec3f triangleNormal(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto t = msh->shapes[0]->triangles[eid];
  return ym::normalize(euv.x * msh->shapes[0]->norm[t.x] +
                   euv.y * msh->shapes[0]->norm[t.y] + euv.z * msh->shapes[0]->norm[t.z]);
}
ym::vec3f lineNormal(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto l = msh->shapes[0]->lines[eid];
  return ym::normalize(euv.x * msh->shapes[0]->norm[l.x] +
                       euv.y * msh->shapes[0]->norm[l.y]);
}
ym::vec3f trianglePosition(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto t = msh->shapes[0]->triangles[eid];
  return (euv.x * msh->shapes[0]->pos[t.x] + euv.y * msh->shapes[0]->pos[t.y] +
         euv.z * msh->shapes[0]->pos[t.z]);
}
ym::vec3f linePosition(const yobj::mesh* msh, int eid, ym::vec3f euv) {
  auto l = msh->shapes[0]->lines[eid];
  return (euv.x * msh->shapes[0]->pos[l.x] + euv.y * msh->shapes[0]->pos[l.y]);
}

ym::vec4f textureLDR(const yobj::mesh* msh, int eid, ym::vec3f euv, yobj::texture* tex){

  auto tr = msh->shapes[0]->triangles[eid];

  float u1 = msh->shapes[0]->texcoord[tr.x].x;
  float v1 = msh->shapes[0]->texcoord[tr.x].y;
  float u2 = msh->shapes[0]->texcoord[tr.y].x;
  float v2 = msh->shapes[0]->texcoord[tr.y].y;
  float u3 = msh->shapes[0]->texcoord[tr.z].x;
  float v3 = msh->shapes[0]->texcoord[tr.z].y;

  float u = euv.x*u1 + euv.y*u2 + euv.z*u3;
  float v = euv.x*v1 + euv.y*v2 + euv.z*v3;

  float s = (fmod(u,1.0f))*tex->width();
  float t = (fmod(v,1.0f))*tex->height();

  auto i = (int)ym::floor(s);
  auto j = (int)ym::floor(t);

  auto i1 = (int)round((i + 1)%tex->width());
  auto j1 = (int)round((j + 1)%tex->height());

  float wi = s - i;
  float wj = t - j;

  return ym::vec4f((1-wi)*(1-wj))*ym::srgb_to_linear(tex->ldr[{i,j}])
         +ym::vec4f(wi*(1-wj))*ym::srgb_to_linear(tex->ldr[{i1,j}])
         +ym::vec4f(wj*(1-wi))*ym::srgb_to_linear(tex->ldr[{i,j1}])
         +ym::vec4f(wi*wj)*ym::srgb_to_linear(tex->ldr[{i1,j1}]);
}

ybvh::scene* make_bvh(yobj::scene* scn) {

  ybvh::scene* bvh_scn = ybvh::make_scene();
  auto shape_map = std::map<yobj::shape*, int>{{nullptr, -1}};

  for (auto mesh : scn->meshes) {

    auto shape=mesh->shapes[0];

    if (!shape->points.empty()) {
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

    if(!shp->points.empty()){
      lights.push_back(ist);
      continue;
    }

    ybvh::add_instance(bvh_scn, ym::to_frame(ist->xform()),
                         shape_map.at(shp));
  }

  ybvh::build_scene_bvh(bvh_scn);
  return bvh_scn;
}

ym::ray3f camera_ray(yobj::camera* cam, float u, float v, float w, float h){

  auto camera_pos = ym::to_frame(cam->xform());

  auto q = camera_pos.o
   + ((u - .5f)*w*camera_pos.x)
   + ((v - .5f)*h*camera_pos.y)
   - (camera_pos.z);

  auto d = q-camera_pos.o;
  ym::ray3f ray = ym::ray3f(camera_pos.o,ym::normalize(d),epsilon);

  return ray;
}

ym::vec4f compute_color(const ybvh::scene* bvh,const ym::vec4f& amb, const yobj::scene* scn, ym::ray3f ray){

  auto intersection = ybvh::intersect_scene(bvh, ray, false);

  ym::vec4f c = ym::vec4f(0,0,0,1);

  if(intersection){

    auto ist = scn->instances[intersection.iid];
    auto msh = ist->msh;
    auto mat = msh->shapes[0]->mat;
    auto kd = mat->kd;
    auto ks = mat->ks;
    auto ist_frame = ym::to_frame(ist->xform());
    auto ns = (mat->rs) ? 2 / (mat->rs * mat->rs) - 2 : 1e6f;

    //triangles
    if(!scn->instances[intersection.iid]->msh->shapes[0]->triangles.empty()){
      ray=ym::transform_ray(ist_frame,ray);
      auto n = triangleNormal(msh,intersection.eid,intersection.euv.xyz());
      auto p = trianglePosition(msh,intersection.eid,intersection.euv.xyz());

      if(mat->kd_txt!= nullptr)
        kd*=textureLDR(msh, intersection.eid, intersection.euv.xyz(), mat->kd_txt).xyz();

      if(mat->ks_txt!= nullptr)
        ks*=textureLDR(msh,intersection.eid,intersection.euv.xyz(), mat->ks_txt).xyz();


      for(auto light:lights){

        auto light_pose = light->translation + light->msh->shapes[0]->pos[0];
        light_pose = ym::transform_point_inverse(ist_frame, light_pose);

        auto l = ym::normalize(light_pose-p);
        auto r = ym::length(light_pose-p);

        ym::ray3f sr = ym::ray3f(p,l,epsilon,r);
        sr=ym::transform_ray(ist_frame,sr);

        auto shadow = ybvh::intersect_scene(bvh, sr, false);
        auto In = light->msh->shapes[0]->mat->ke / (r * r);
        if(!shadow) {
          auto v = ym::normalize(ray.o-p);
          auto h = ym::normalize((v+l));

          c.xyz() += kd * In * ym::max(.0f, ym::dot(n, l))
                   + ks * In * ym::pow(ym::max(.0f,ym::dot(n,h)),ns);
        }
      }

      //reflection
      if(mat->kr.x!=0 && mat->kr.y!=0 && mat->kr.z!=0)
      {
        ym::ray3f reflectionRay = ym::ray3f();
        reflectionRay.o=p;
        reflectionRay.d= (n*(ym::dot(n,-ray.d) * ym::vec3f(2)))+ray.d;
        reflectionRay.tmin=epsilon;
        c.xyz() += mat->kr*compute_color(bvh,amb,scn, reflectionRay).xyz();
      }

      //trasparenza
      if((mat->opacity>=0) && (mat->opacity<1)){
        c.xyz() *= ym::vec3f(mat->opacity);
        c.xyz() += ym::vec3f(1-mat->opacity)* compute_color(bvh,amb,scn,ym::ray3f(p,ray.d,epsilon)).xyz();
      }
    }

    //lines
    else{
      auto n = lineNormal(msh,intersection.eid,intersection.euv.xyz());
      auto p = linePosition(msh,intersection.eid,intersection.euv.xyz());

      for(auto light:lights){

        auto l = normalize(light->msh->shapes[0]->pos[0]-p);
        auto r = length(light->msh->shapes[0]->pos[0]-p);

        auto v = ym::normalize(ray.o-p);
        auto h = ym::normalize((v+l));

        ym::ray3f sr = ym::ray3f{p,l,epsilon,r};
        auto shadow = ybvh::intersect_scene(bvh, sr, false);

        if(!shadow) {
          auto In = light->msh->shapes[0]->mat->ke / (r * r);
          c.xyz() += kd * In * ym::sqrt(1-ym::abs(ym::dot(n,l)))
                  + ks * In * ym::pow(sqrt(1-ym::abs(ym::dot(n,h))),ns);
        }
      }
    }
    c.xyz()+=kd*amb.xyz();
  }
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
          img[{i, (height-1-j)}] +=  compute_color(bvh,amb,scn,ray);
        }
      }
      img[{i, (height-1-j)}] /= {norm,norm,norm,1};
    }
  }

  return img;
}



ym::image4f raytrace_mt(const yobj::scene* scn, const ybvh::scene* bvh,
                        const ym::vec4f& amb, int resolution, int samples) {
  return {raytrace(scn,bvh,amb,resolution,samples)};
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
