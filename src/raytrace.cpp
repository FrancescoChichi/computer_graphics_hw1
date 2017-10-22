#include <thread>
#include "yocto_bvh.h"
#include "yocto_img.h"
#include "yocto_math.h"
#include "yocto_obj.h"
#include "yocto_utils.h"

ybvh::scene* make_bvh(yobj::scene* scn) {
    // YOUR CODE GOES HERE ----------------------
    return nullptr;
}

ym::image4f raytrace(const yobj::scene* scn, const ybvh::scene* bvh,
    const ym::vec3f& amb, int resolution, int samples) {
    // YOUR CODE GOES HERE ----------------------
    return {};
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
