#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

using namespace rerun::demo;

int main()
{
    const auto rec = rerun::RecordingStream("rerun_example_cpp");

    rec.spawn().exit_on_failure();

    std::vector<rerun::Position3D> points = grid3d<rerun::Position3D, float>(-10.0f, 10.0f, 10);
    std::vector<rerun::Color> colors = grid3d<rerun::Color, uint8_t>(0, 255, 10);

    rec.log("my_points", rerun::Points3D(points).with_colors(colors).with_radii({0.5f}));
}