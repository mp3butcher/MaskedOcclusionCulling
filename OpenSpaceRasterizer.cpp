#include "OpenSpaceRasterizer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "MaskedOcclusionCulling.h"

namespace {
inline int clampInt(int value, int lo, int hi) {
    return std::max(lo, std::min(value, hi));
}
}

OpenSpaceRasterizer::OpenSpaceRasterizer(unsigned width, unsigned height)
    : rasterizer_(nullptr), width_(width), height_(height), openDepth_(static_cast<std::size_t>(width) * height, 0.0f) {
    if (width == 0 || height == 0 || (width & 7u) != 0 || (height & 3u) != 0)
        throw std::invalid_argument("OpenSpaceRasterizer resolution must be width % 8 == 0 and height % 4 == 0");

    rasterizer_ = MaskedOcclusionCulling::Create(MaskedOcclusionCulling::SSE2);
    rasterizer_->SetResolution(width_, height_);
    ClearBuffer();
}

OpenSpaceRasterizer::~OpenSpaceRasterizer() {
    if (rasterizer_) MaskedOcclusionCulling::Destroy(rasterizer_);
}

void OpenSpaceRasterizer::ClearBuffer() {
    rasterizer_->ClearBuffer();
    std::fill(openDepth_.begin(), openDepth_.end(), 0.0f);
}

MaskedOcclusionCulling::CullingResult OpenSpaceRasterizer::RasterizeTriangles(
    const float* inVtx, const unsigned int* inTris, int nTris,
    const float* modelToClipMatrix,
    MaskedOcclusionCulling::BackfaceWinding bfWinding,
    MaskedOcclusionCulling::ClipPlanes clipPlaneMask,
    const MaskedOcclusionCulling::VertexLayout& vtxLayout) {
    const MaskedOcclusionCulling::CullingResult result = rasterizer_->RenderTriangles(
        inVtx, inTris, nTris, modelToClipMatrix, bfWinding, clipPlaneMask, vtxLayout);
    rebuildOpenDepth();
    return result;
}

void OpenSpaceRasterizer::rebuildOpenDepth() {
    // MOC's cleared depth is zero and rendered clip-space samples have depth
    // 1/w. ComputePixelDepthBuffer expands its hierarchical representation;
    // non-zero values therefore form the accumulated open-space coverage mask.
    rasterizer_->ComputePixelDepthBuffer(openDepth_.data(), false);
}

bool OpenSpaceRasterizer::TestRect(float xmin, float ymin, float xmax, float ymax, float wmin) const {
    if (xmax <= xmin || ymax <= ymin || wmin <= 0.0f) return false;

    const int x0 = clampInt(static_cast<int>(std::floor((xmin * 0.5f + 0.5f) * width_)), 0, static_cast<int>(width_));
    const int x1 = clampInt(static_cast<int>(std::ceil ((xmax * 0.5f + 0.5f) * width_)), 0, static_cast<int>(width_));
    const int y0 = clampInt(static_cast<int>(std::floor((ymin * 0.5f + 0.5f) * height_)), 0, static_cast<int>(height_));
    const int y1 = clampInt(static_cast<int>(std::ceil ((ymax * 0.5f + 0.5f) * height_)), 0, static_cast<int>(height_));
    if (x0 >= x1 || y0 >= y1) return false;

    const float objectDepth = 1.0f / wmin;
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            const float openDepth = openDepth_[static_cast<std::size_t>(y) * width_ + x];
            // An empty-space sample in front of, or at, the object's nearest
            // point is sufficient to keep the object conservatively.
            if (openDepth > 0.0f && openDepth >= objectDepth) return true;
        }
    }
    return false;
}
