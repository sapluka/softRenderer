#include "camera.h"
#include "scene.h"
#include "transform.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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

    return failed == 0 ? 0 : 1;
}

