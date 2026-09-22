#include "OpenSpaceRasterizer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
inline int clampPixel(float ndc, unsigned extent) {
    return std::max(0, std::min(static_cast<int>(extent),
        static_cast<int>(std::floor((ndc + 1.0f) * static_cast<float>(extent) * 0.5f))));
}
}

OpenSpaceRasterizer::OpenSpaceRasterizer(unsigned width, unsigned height)
    : moc_(nullptr), width_(width), height_(height),
      openDepth_(static_cast<std::size_t>(width) * height, 0.0f) {
    if (width == 0 || height == 0 || (width & 7u) != 0 || (height & 3u) != 0)
        throw std::invalid_argument("OpenSpaceRasterizer resolution must be width % 8 == 0 and height % 4 == 0");

    // This is a private MOC buffer used exclusively for empty-space geometry.
    // No solid object is ever submitted to it, so its normal occluder update
    // path becomes the desired inverse-coverage max(1/w) update.
    moc_ = MaskedOcclusionCulling::Create(MaskedOcclusionCulling::AVX512);
    moc_->SetResolution(width_, height_);
    ClearBuffer();
}

OpenSpaceRasterizer::~OpenSpaceRasterizer() {
    if (moc_) MaskedOcclusionCulling::Destroy(moc_);
}

void OpenSpaceRasterizer::ClearBuffer() {
    moc_->ClearBuffer();
    std::fill(openDepth_.begin(), openDepth_.end(), 0.0f);
}

MaskedOcclusionCulling::CullingResult OpenSpaceRasterizer::RasterizeTriangles(
    const float* inVtx, const unsigned int* inTris, int nTris,
    const float* modelToClipMatrix,
    MaskedOcclusionCulling::BackfaceWinding bfWinding,
    MaskedOcclusionCulling::ClipPlanes clipPlaneMask,
    const MaskedOcclusionCulling::VertexLayout& vtxLayout) {
    const auto result = moc_->RenderTriangles(
        inVtx, inTris, nTris, modelToClipMatrix, bfWinding, clipPlaneMask, vtxLayout);
    readDepth();
    return result;
}

void OpenSpaceRasterizer::readDepth() {
    // MOC stores reversed depth and updates nearer samples with greater values.
    // The expanded image therefore already has the required max(1/w) semantics.
    moc_->ComputePixelDepthBuffer(openDepth_.data(), false);
}

bool OpenSpaceRasterizer::TestRect(float xmin, float ymin, float xmax, float ymax, float wmin) const {
    if (wmin <= 0.0f || xmax <= xmin || ymax <= ymin) return false;

    const int x0 = clampPixel(xmin, width_);
    const int x1 = std::max(0, std::min(static_cast<int>(width_),
        static_cast<int>(std::ceil((xmax + 1.0f) * width_ * 0.5f))));
    const int y0 = clampPixel(ymin, height_);
    const int y1 = std::max(0, std::min(static_cast<int>(height_),
        static_cast<int>(std::ceil((ymax + 1.0f) * height_ * 0.5f))));
    if (x0 >= x1 || y0 >= y1) return false;

    const float objectDepth = 1.0f / wmin;
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            if (openDepth_[static_cast<std::size_t>(y) * width_ + x] >= objectDepth)
                return true;
        }
    }
    return false;
}
