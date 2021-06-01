#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/TextureRenderTargetCube.h"
#include "common/WorkerThread.hpp"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SceneCaptureComponentCube.h"
#include "Engine/GameViewportClient.h"
#include "common/Common.hpp"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"

// Cube.
template < typename FromT, typename ToT >
void copyFromTArray2TArray( const FromT& from, ToT& to ) {
    const int N = from.Num();

    to.Empty();
    to.SetNumUninitialized(N);

    auto ptr_from = from.GetData();
    auto ptr_to = to.GetData();

    for ( int i = 0; i < N; i++ ) {
        *ptr_to++ = *ptr_from++;
    }
}

template < typename FromArrayT, typename ToArrayT >
bool compressTArrayAsPng32bit( const FromArrayT& from_array, ToArrayT& to_array,
	int32 width, int32 height, int32 compression=100 )
{
	IImageWrapperModule& image_wrapper_module = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
	TSharedPtr<IImageWrapper> image_wrapper = image_wrapper_module.CreateImageWrapper(EImageFormat::PNG);

	if ( image_wrapper->SetRaw( from_array.GetData(), from_array.GetAllocatedSize(), width, height, ERGBFormat::BGRA, 8 ) )
	{
		to_array = image_wrapper->GetCompressed(compression);
        return true;
	} else {
        return false;
    }

    // Comment the following line introduces memory leak?
	// ImageWrapper.Reset();
}

class RenderRequest : public FRenderCommand
{
public:
    struct RenderParams {
        USceneCaptureComponent2D * const render_component;
        UTextureRenderTarget2D* render_target;
        bool pixels_as_float;
        bool compress;

        // Cube.
        USceneCaptureComponentCube* const render_component_cube;
        UTextureRenderTargetCube* render_target_cube;
        bool nearest_cube = false;
        bool is_cube; // Set true for cube.

        RenderParams(USceneCaptureComponent2D * render_component_val, UTextureRenderTarget2D* render_target_val, bool pixels_as_float_val, bool compress_val)
            : render_component(render_component_val), render_target(render_target_val), pixels_as_float(pixels_as_float_val), compress(compress_val)
            , render_component_cube(nullptr), render_target_cube(nullptr), is_cube(false)
        {
        }

        // Overload for cube.
        RenderParams( USceneCaptureComponentCube* render_component_val, UTextureRenderTargetCube* render_target_val, bool pixels_as_float_val, bool compress_val, bool nearest_cube_val=false )
        : render_component(nullptr), render_target(nullptr), pixels_as_float(pixels_as_float_val), compress(compress_val)
        , render_component_cube(render_component_val), render_target_cube(render_target_val), nearest_cube(nearest_cube_val), is_cube(true)
        {
        }
    };

    struct RenderResult {
        // Cube.
        RenderResult() {
            bmp_float_6.SetNum(6, true);
        }

        TArray<uint8> image_data_uint8;
        TArray<float> image_data_float;

        TArray<FColor> bmp;
        TArray<FFloat16Color> bmp_float;

        // Cube.
        TArray64<uint8> cube_raw;
        TArray64<uint8> cube_image_data;
        TArray<TArray<FFloat16Color>> bmp_float_6;

        int width;
        int height;

        msr::airlib::TTimePoint time_stamp;
    };

private:
    static FReadSurfaceDataFlags setupRenderResource(const FTextureRenderTargetResource* rt_resource, const RenderParams* params, RenderResult* result, FIntPoint& size);
    
    // Cube.
    bool unWarpTextureRenderTargetCube( const UTextureRenderTargetCube* TRTCube, TArray64<uint8>& OutData );

    std::shared_ptr<RenderParams>* params_;
    std::shared_ptr<RenderResult>* results_;
    unsigned int req_size_;

    std::shared_ptr<msr::airlib::WorkerThreadSignal> wait_signal_;

    bool saved_DisableWorldRendering_ = false;
    UGameViewportClient * const game_viewport_;
    FDelegateHandle end_draw_handle_;
    std::function<void()> query_camera_pose_cb_;

public:
    RenderRequest(UGameViewportClient * game_viewport, std::function<void()>&& query_camera_pose_cb);
    ~RenderRequest();

    void DoTask(ENamedThreads::Type CurrentThread, const FGraphEventRef& MyCompletionGraphEvent)
    {
        ExecuteTask();
    } 

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(RenderRequest, STATGROUP_RenderThreadCommands);
    }

    // read pixels from render target using render thread, then compress the result into PNG
    // argument on the thread that calls this method.
    void getScreenshot(
        std::shared_ptr<RenderParams> params[], std::vector<std::shared_ptr<RenderResult>>& results, unsigned int req_size, bool use_safe_method);

    void ExecuteTask();
};
