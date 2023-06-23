#include "OptimalTrajectoryGenerator.h"
#include "SwervePathBuilderWrap.h"
#include "path/SwervePathBuilder.h"
#include "TrajectoryGenerationException.h"
#include "trajectory/HolonomicTrajectory.h"
#include <napi.h>
#include <math.h>
#include <optional>
#include <utility>

class GenerationWorker : public Napi::AsyncWorker {
public:
  GenerationWorker(const Napi::Env& env, trajopt::SwervePathBuilder path)
      : Napi::AsyncWorker(env),
      deferred(Napi::Promise::Deferred::New(env)),
      path(std::move(path)) {}

  // Executed inside the worker-thread.
  // It is not safe to access JS engine data structure
  // here, so everything we need for input and output
  // should go on `this`.
  void Execute() override {
    try {
      trajectory = trajopt::HolonomicTrajectory(trajopt::OptimalTrajectoryGenerator::Generate(path));
    } catch (const trajopt::TrajectoryGenerationException& e) {
      SetError(e.what());
      return;
    }
    // you could handle errors as well
    // throw std::runtime_error("test error");
    // or like
    // Napi::AsyncWorker::SetError
    // Napi::AsyncWorker::SetError("test error");
  }

  // Executed when the async work is complete
  // this function will be run inside the main event loop
  // so it is safe to use JS engine data again
  void OnOK() override {
    auto napiSamples = Napi::Array::New(Env());
    size_t sampCnt = trajectory->samples.size();
    for (size_t idx = 0; idx < sampCnt; ++idx) {
      Napi::Object napiSample = Napi::Object::New(Env());
      napiSample.Set("timestamp", trajectory->samples.at(idx).timestamp);
      napiSample.Set("x", trajectory->samples.at(idx).x);
      napiSample.Set("y", trajectory->samples.at(idx).y);
      napiSample.Set("heading", trajectory->samples.at(idx).heading);
      napiSample.Set("velocityX", trajectory->samples.at(idx).velocityX);
      napiSample.Set("velocityY", trajectory->samples.at(idx).velocityY);
      napiSample.Set("angularVelocity", trajectory->samples.at(idx).angularVelocity);
      napiSamples.Set(idx, napiSample);
    }
    deferred.Resolve(napiSamples);
  }

  void OnError(const Napi::Error& error) override {
    deferred.Reject(error.Value());
  }

  Napi::Promise GetPromise() { return deferred.Promise(); }

private:
  Napi::Promise::Deferred deferred;
  trajopt::SwervePathBuilder path;
  std::optional<trajopt::HolonomicTrajectory> trajectory = std::nullopt;
};

Napi::Value CalculatePiAsync(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  trajopt::SwervePathBuilder path;

  GenerationWorker* worker = new GenerationWorker(env, path);

  auto promise = worker->GetPromise();

  worker->Queue();

  return promise;
}