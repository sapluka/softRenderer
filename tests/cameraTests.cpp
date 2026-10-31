#include "camera.h"
#include "scene.h"
#include "transform.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

struct RasterizerMSAATestAccess {
    static std::size_t coveredSamples(const rasterizer& renderer) {
        std::size_t covered = 0;
        for (const auto& row : renderer.subZBuffer)
            for (const auto& samples : row)
                for (float depth : samples) covered += std::isfinite(depth);
        return covered;
    }
};

namespace {
using Eigen::Vector3f;
constexpr double radiansPerPixel = 0.005;
constexpr double halfPi = 1.57079632679489661923;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void requireNear(const Vector3f& actual, const Vector3f& expected,
                 const std::string& message, float tolerance = 2e-4f) {
    require(actual.allFinite() && (actual - expected).norm() < tolerance, message);
}

void checkPose(const camera& camera, const Vector3f& target, float radius) {
    const Vector3f offset = camera.getPosition() - target;
    requireNear(camera.getTarget(), target, "Orbit changed the target.");
    require(offset.allFinite() && std::abs(offset.norm() - radius) < 1e-3f,
            "Orbit changed the radius.");
    require(camera.getUp().allFinite() && std::abs(camera.getUp().norm() - 1.0f) < 2e-4f,
            "Camera up must remain finite and unit length.");
    require(std::abs(camera.getUp().dot(offset.normalized())) < 2e-4f,
            "Camera up must remain perpendicular to the viewing direction.");
    requireNear(camera.getDirection(), -offset.normalized(),
                "Camera direction did not follow its new position.");
}

struct OrbitScene {
    objLoader model;
    shadeModel shader;
    rasterizer rasterizer_;
    camera camera_;
    scene scene_;

    OrbitScene(const Vector3f& eye, const Vector3f& target,
               const Vector3f& up = Vector3f::UnitY())
        : rasterizer_(shader), camera_(eye, target, up), scene_(model, rasterizer_, camera_) {}
};

void checkFraming(const camera& camera, const Vector3f& minimum, const Vector3f& maximum) {
    require(camera.getPosition().allFinite() && camera.getNearPlane() < 0.0f
                && camera.getFarPlane() < camera.getNearPlane(),
            "Automatic framing produced an invalid eye or clipping range.");
    for (int corner = 0; corner < 8; ++corner) {
        Vector3f point;
        for (int axis = 0; axis < 3; ++axis)
            point[axis] = corner & (1 << axis) ? maximum[axis] : minimum[axis];
        float clipW;
        const Vector3f screen = Transform::transform(point, camera.getPosition(), camera.getTarget(),
            camera.getUp(), clipW, camera.getNearPlane(), camera.getFarPlane());
        require(clipW > 0.0f && screen.allFinite()
                    && screen.x() >= 0.0f && screen.x() <= SCREEN_WIDTH
                    && screen.y() >= 0.0f && screen.y() <= SCREEN_HEIGHT
                    && screen.z() >= -1.0001f && screen.z() <= 1.0001f,
                "A model corner fell outside the automatically fitted view.");
    }
}

void checkSphereDepth(const camera& camera, const Vector3f& center, float radius) {
    require(camera.getNearPlane() < 0.0f && camera.getFarPlane() < camera.getNearPlane(),
            "Moving the camera invalidated its clipping range.");
    const Vector3f forward = (camera.getTarget() - camera.getPosition()).normalized();
    for (float side : {-1.0f, 1.0f}) {
        float clipW;
        const Vector3f screen = Transform::transform(center + side * radius * forward,
            camera.getPosition(), camera.getTarget(), camera.getUp(), clipW,
            camera.getNearPlane(), camera.getFarPlane());
        require(clipW > 0.0f && screen.allFinite()
                    && screen.z() >= -1.0001f && screen.z() <= 1.0001f,
                "A camera move clipped the retained model sphere in depth.");
    }
}

class CornellSizedBox {
public:
    CornellSizedBox() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path = std::filesystem::temp_directory_path()
            / ("renderer_camera_box_" + std::to_string(stamp) + ".obj");
        std::ofstream stream(path);
        stream << "v 0 0 0\nv 556 0 0\nv 556 548.8 0\nv 0 548.8 0\n"
                  "v 0 0 559.2\nv 556 0 559.2\nv 556 548.8 559.2\nv 0 548.8 559.2\n"
                  "f 1 4 3 2\nf 5 6 7 8\nf 1 2 6 5\nf 4 8 7 3\n"
                  "f 1 5 8 4\nf 2 3 7 6\n";
        stream.close();
        require(static_cast<bool>(stream), "Cannot write Cornell-sized camera fixture.");
        require(model.loadObj(path), "Cannot load Cornell-sized camera fixture: " + model.error());
    }

    ~CornellSizedBox() {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }

    objLoader model;

private:
    std::filesystem::path path;
};
} // namespace

int main() {
    unsigned int failed = 0;
    const auto run = [&](const std::string& name, const std::function<void()>& test) {
        try {
            test();
            std::cout << "PASS " << name << '\n';
        } catch (const std::exception& error) {
            ++failed;
            std::cerr << "FAIL " << name << ": " << error.what() << '\n';
        }
    };

    run("horizontal quarter orbit around a non-origin target", [] {
        const Vector3f target(2.0f, -3.0f, 4.0f);
        OrbitScene fixture(target + Vector3f(0.0f, 0.0f, 5.0f), target);
        // Dragging right moves the eye to the screen's right under the existing
        // projection convention, which corresponds to negative world X here.
        fixture.scene_.rotateCamera(80.0, 90.0, 80.0 + halfPi / radiansPerPixel, 90.0);
        requireNear(fixture.camera_.getPosition(), target + Vector3f(-5.0f, 0.0f, 0.0f),
                    "A quarter horizontal orbit ended at the wrong location.");
        requireNear(fixture.camera_.getUp(), Vector3f::UnitY(),
                    "A horizontal orbit unexpectedly rolled the camera.");
        checkPose(fixture.camera_, target, 5.0f);
    });

    run("vertical quarter orbit transports up through the pole", [] {
        const Vector3f target(-2.0f, 1.0f, -7.0f);
        OrbitScene fixture(target + Vector3f(0.0f, 0.0f, 3.0f), target);
        fixture.scene_.rotateCamera(0.0, 0.0, 0.0, halfPi / radiansPerPixel);
        requireNear(fixture.camera_.getPosition(), target + Vector3f(0.0f, 3.0f, 0.0f),
                    "A quarter vertical orbit ended at the wrong location.");
        requireNear(fixture.camera_.getUp(), -Vector3f::UnitZ(),
                    "Up did not rotate with the eye at the world-Y pole.");
        checkPose(fixture.camera_, target, 3.0f);

        fixture.scene_.rotateCamera(0.0, 0.0, 0.0, 2.0);
        const Vector3f afterPoleUp = fixture.camera_.getUp();
        require(afterPoleUp.dot(-Vector3f::UnitZ()) > 0.999f,
                "Up flipped when crossing the world-Y pole.");
        require(afterPoleUp.y() < 0.0f,
                "Crossing the pole should allow camera up to point below world Y.");
        checkPose(fixture.camera_, target, 3.0f);
        const Eigen::Matrix4f view = Transform::getViewMatrix(
            fixture.camera_.getPosition(), target, fixture.camera_.getUp());
        require(view.allFinite(), "View became invalid after crossing the pole.");
    });

    run("oblique drag follows the specified tangent direction", [] {
        const Vector3f target(1.0f, 2.0f, 3.0f);
        OrbitScene fixture(target + Vector3f(0.0f, 0.0f, 4.0f), target);
        fixture.scene_.rotateCamera(10.0, 20.0, 70.0, 100.0);
        const Vector3f offset = fixture.camera_.getPosition() - target;
        require(offset.x() < 0.0f && offset.y() > 0.0f && offset.z() > 0.0f,
                "Oblique orbit moved into the wrong quadrant.");
        require(std::abs(offset.y() / -offset.x() - 4.0f / 3.0f) < 2e-4f,
                "Oblique orbit did not follow the screen drag direction.");
        require(std::abs(std::acos(offset.normalized().z()) - 0.5f) < 2e-4f,
                "Drag length did not control the angular distance.");
        checkPose(fixture.camera_, target, 4.0f);
    });

    run("rotated up controls later drags", [] {
        OrbitScene fixture(Vector3f(0.0f, 0.0f, 5.0f), Vector3f::Zero(), Vector3f::UnitX());
        fixture.scene_.rotateCamera(0.0, 0.0, halfPi / radiansPerPixel, 0.0);
        requireNear(fixture.camera_.getPosition(), Vector3f(0.0f, 5.0f, 0.0f),
                    "Horizontal drag ignored the camera's rolled frame.");
        requireNear(fixture.camera_.getUp(), Vector3f::UnitX(),
                    "Orbit unexpectedly replaced custom up with world up.");
        checkPose(fixture.camera_, Vector3f::Zero(), 5.0f);
    });

    run("successive drags preserve radius and can be reversed", [] {
        const Vector3f target(3.0f, -1.0f, 8.0f);
        const Vector3f eye = target + Vector3f(0.0f, 0.0f, 7.0f);
        OrbitScene fixture(eye, target);
        std::vector<std::pair<double, double>> drags;
        for (int i = 0; i < 120; ++i) {
            const double dx = 11.0 * std::sin(i * 0.7);
            const double dy = 17.0 * std::cos(i * 0.3);
            drags.emplace_back(dx, dy);
            fixture.scene_.rotateCamera(0.0, 0.0, dx, dy);
            checkPose(fixture.camera_, target, 7.0f);
        }
        for (auto drag = drags.rbegin(); drag != drags.rend(); ++drag) {
            fixture.scene_.rotateCamera(0.0, 0.0, -drag->first, -drag->second);
            checkPose(fixture.camera_, target, 7.0f);
        }
        requireNear(fixture.camera_.getPosition(), eye,
                    "Reversing a drag sequence did not restore the eye.", 1e-3f);
        requireNear(fixture.camera_.getUp(), Vector3f::UnitY(),
                    "Reversing a drag sequence did not restore up.", 1e-3f);
    });

    run("zero drag and coincident eye-target are safe", [] {
        OrbitScene fixture(Vector3f(1.0f, 2.0f, 8.0f), Vector3f(1.0f, 2.0f, 3.0f));
        const Vector3f eye = fixture.camera_.getPosition();
        const Vector3f up = fixture.camera_.getUp();
        fixture.scene_.rotateCamera(40.0, 50.0, 40.0, 50.0);
        requireNear(fixture.camera_.getPosition(), eye, "Zero drag changed the eye.");
        requireNear(fixture.camera_.getUp(), up, "Zero drag changed up.");

        const Vector3f point(1.0f, -2.0f, 3.0f);
        OrbitScene coincident(point, point);
        coincident.scene_.rotateCamera(0.0, 0.0, 100.0, 50.0);
        requireNear(coincident.camera_.getPosition(), point,
                    "A coincident eye-target orbit changed the position.");
        requireNear(coincident.camera_.getUp(), Vector3f::UnitY(),
                    "A coincident eye-target orbit invalidated up.");
    });

    run("view matrix and screen transform honor camera roll", [] {
        const Vector3f eye(0.0f, 0.0f, 5.0f);
        const Vector3f target = Vector3f::Zero();
        const Eigen::Matrix4f defaultView = Transform::getViewMatrix(eye, target);
        require((defaultView - Transform::getViewMatrix(eye, target, Vector3f::UnitY())).norm() < 1e-5f,
                "Default up changed existing getViewMatrix calls.");
        const Eigen::Matrix4f rolledView = Transform::getViewMatrix(eye, target, Vector3f::UnitX());
        const Eigen::Vector4f rolledPoint = rolledView * Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
        requireNear(rolledPoint.head<3>(), Vector3f(0.0f, 1.0f, -5.0f),
                    "View matrix ignored a rolled up vector.");
        const Eigen::Vector4f eyeInView = rolledView * Eigen::Vector4f(0.0f, 0.0f, 5.0f, 1.0f);
        requireNear(eyeInView.head<3>(), Vector3f::Zero(), "View matrix did not map eye to origin.");

        const Vector3f center = Transform::transform(target, eye, target);
        const Vector3f unrolled = Transform::transform(Vector3f::UnitX(), eye, target);
        const Vector3f rolled = Transform::transform(Vector3f::UnitX(), eye, target, Vector3f::UnitX());
        requireNear(unrolled, Transform::transform(Vector3f::UnitX(), eye, target, Vector3f::UnitY()),
                    "Default up changed existing transform calls.");
        require(unrolled.x() > center.x() && std::abs(unrolled.y() - center.y()) < 1e-4f,
                "World right did not project to screen right.");
        const Vector3f upper = Transform::transform(Vector3f::UnitY(), eye, target);
        require(std::abs(upper.x() - center.x()) < 1e-4f && upper.y() < center.y(),
                "World up did not project to screen up.");
        require(std::abs(rolled.x() - center.x()) < 1e-4f && rolled.y() < center.y(),
                "Screen transform did not apply camera roll.");
    });

    run("prepared transforms preserve clip w and viewport coordinates for both passes", [] {
        for (const Vector3f up : {Vector3f::UnitY().eval(), Vector3f::UnitX().eval()}) {
            const Vector3f eye(4.0f, 3.0f, 11.0f);
            const Vector3f target(1.0f, -2.0f, 3.0f);
            const Eigen::Matrix4f view = Transform::getViewMatrix(eye, target, up);
            const Eigen::Matrix4f projection = Transform::getProjectionMatrix(-.25f, -250.0f);
            for (int scale : {1, 2}) {
                const int width = SCREEN_WIDTH * scale;
                const int height = SCREEN_HEIGHT * scale;
                const Eigen::Matrix4f viewport = Transform::getViewportMatrix(width, height);
                const Transform prepared(eye, target, up, width, height, -.25f, -250.0f);
                require(prepared.getBufferWidth() == width && prepared.getBufferHeight() == height,
                        "A prepared transform lost its output dimensions.");
                for (const Vector3f point : {target, Vector3f(-2, 1, -4), Vector3f(7, -3, 2)}) {
                    const Eigen::Vector4f clip = projection * view * Eigen::Matrix4f::Identity()
                                              * Eigen::Vector4f(point.x(), point.y(), point.z(), 1.0f);
                    const Eigen::Vector4f ndc(clip.x() / clip.w(), clip.y() / clip.w(), clip.z() / clip.w(), 1.0f);
                    const Eigen::Vector4f expected = viewport * ndc;
                    float clipW;
                    const Vector3f actual = prepared.apply(point, clipW);
                    requireNear(actual, expected.head<3>(), "Prepared matrices changed projected coordinates.");
                    require(std::abs(clipW - clip.w()) < 2e-5f,
                            "Prepared matrices lost the original clip w needed for interpolation.");
                    float legacyW;
                    requireNear(actual, Transform::transform(point, eye, target, up, legacyW,
                                                            -.25f, -250.0f, width, height),
                                "The compatibility transform differs from the prepared transform.");
                }
            }
        }
    });

    run("parallel up has a finite orthonormal fallback", [] {
        const Vector3f eye(2.0f, 6.0f, -3.0f);
        const Vector3f target(2.0f, 1.0f, -3.0f);
        const Eigen::Matrix4f view = Transform::getViewMatrix(eye, target, Vector3f::UnitY());
        const Eigen::Matrix3f orientation = view.block<3, 3>(0, 0);
        require(view.allFinite(), "Parallel up generated a non-finite view matrix.");
        require((orientation * orientation.transpose() - Eigen::Matrix3f::Identity()).norm() < 2e-4f,
                "Fallback camera basis is not orthonormal.");
        require(std::abs(orientation.determinant() - 1.0f) < 2e-4f,
                "Fallback camera basis changed handedness.");
    });

    run("automatic framing fits translated models at different scales and flat models", [] {
        for (float scale : {.001f, 1.0f, 1000.0f}) {
            const Vector3f center = scale * Vector3f(3.0f, -2.0f, 7.0f);
            const Vector3f extent = scale * Vector3f(3.0f, 2.0f, 4.0f);
            const Vector3f minimum = center - extent;
            const Vector3f maximum = center + extent;
            const camera fitted(minimum, maximum);
            requireNear(fitted.getTarget(), center, "Automatic framing ignored the model center.");
            require(fitted.getPosition().z() > center.z(), "Initial eye was not placed along positive Z.");
            checkFraming(fitted, minimum, maximum);
        }
        const Vector3f minimum(-500.0f, -400.0f, 25.0f);
        const Vector3f maximum(500.0f, 400.0f, 25.0f);
        checkFraming(camera(minimum, maximum), minimum, maximum);
    });

    run("fitted camera retains depth coverage through orbit zoom and target changes", [] {
        const Vector3f center(278.0f, 274.4f, 279.6f);
        const Vector3f extent(278.0f, 274.4f, 279.6f);
        camera fitted(center - extent, center + extent);
        objLoader model;
        shadeModel shader;
        rasterizer renderer(shader);
        scene scene(model, renderer, fitted);
        const Vector3f initialEye = fitted.getPosition();
        scene.rotateCamera(0.0, 0.0, 140.0, 170.0);
        require((fitted.getPosition() - initialEye).norm() > 1.0f, "Framed camera did not orbit.");
        checkFraming(fitted, center - extent, center + extent);
        fitted.setPosition(fitted.getTarget() + (fitted.getPosition() - fitted.getTarget()) * 1.8f);
        checkSphereDepth(fitted, center, extent.norm());
        fitted.setPosition(fitted.getTarget() + (fitted.getPosition() - fitted.getTarget()) * .45f);
        checkSphereDepth(fitted, center, extent.norm());
        const Vector3f eye = fitted.getPosition();
        fitted.setTarget(center + Vector3f(100.0f, -50.0f, 0.0f));
        requireNear(fitted.getPosition(), eye, "Updating the clipping range reset the camera eye.");
        checkSphereDepth(fitted, center, extent.norm());
        fitted.setPosition(fitted.getTarget() + (fitted.getPosition() - fitted.getTarget()) * .1f);
        require((fitted.getPosition() - center).norm() < extent.norm(),
                "The zoom fixture did not enter the model sphere.");
        require(fitted.getNearPlane() < 0.0f && fitted.getFarPlane() < fitted.getNearPlane()
                    && Transform::getProjectionMatrix(fitted.getNearPlane(), fitted.getFarPlane()).allFinite(),
                "Zooming inside the model sphere invalidated the projection.");
    });

    run("default cameras and projection calls retain the original clipping range", [] {
        camera original;
        require(original.getNearPlane() == z_near && original.getFarPlane() == z_far,
                "The default camera changed the original clipping range.");
        original.setPosition(Vector3f(0.0f, 0.0f, 5.0f));
        original.setTarget(Vector3f::Zero());
        require(original.getNearPlane() == z_near && original.getFarPlane() == z_far,
                "Moving an unfitted camera enabled automatic clipping.");
        require((Transform::getProjectionMatrix()
                    - Transform::getProjectionMatrix(z_near, z_far)).norm() < 1e-6f,
                "Default projection changed when clipping parameters became configurable.");
        float clipW;
        requireNear(Transform::transform(Vector3f::Zero(), original.getPosition(), original.getTarget()),
            Transform::transform(Vector3f::Zero(), original.getPosition(), original.getTarget(),
                original.getUp(), clipW, z_near, z_far),
            "Default screen transform differs from explicit original clipping planes.");
    });

    run("Cornell-sized geometry rasterizes with the fitted clipping range", [] {
        const CornellSizedBox box;
        const camera fitted(Vector3f::Zero(), Vector3f(556.0f, 548.8f, 559.2f));
        shadeModel shader;
        rasterizer renderer(shader);
        renderer.initRasterizer();
        renderer.rasterize(box.model, fitted.getPosition(), fitted.getTarget(), fitted.getUp());
        require(RasterizerMSAATestAccess::coveredSamples(renderer) == 0,
                "Cornell-sized fixture no longer reproduces the original far-plane clipping.");
        renderer.clear();
        renderer.rasterize(box.model, fitted.getPosition(), fitted.getTarget(), fitted.getUp(),
                           fitted.getNearPlane(), fitted.getFarPlane());
        require(RasterizerMSAATestAccess::coveredSamples(renderer) > 0,
                "Automatic camera clipping still rejected every Cornell-sized sample.");
    });

    return failed == 0 ? 0 : 1;
}

