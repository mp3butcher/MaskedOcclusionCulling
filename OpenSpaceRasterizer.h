#pragma once

#include <cstddef>
#include <vector>
#include "MaskedOcclusionCulling.h"

// Empty-space coverage buffer backed by a dedicated MOC instance. Since this
// instance receives only empty-cell geometry, MOC's normal reversed-depth
// update is exactly the desired max(1/w) open-space operation.
class OpenSpaceRasterizer {
public:
    OpenSpaceRasterizer(unsigned width, unsigned height);
    ~OpenSpaceRasterizer();

    OpenSpaceRasterizer(const OpenSpaceRasterizer&) = delete;
    OpenSpaceRasterizer& operator=(const OpenSpaceRasterizer&) = delete;

    void ClearBuffer();

    MaskedOcclusionCulling::CullingResult RasterizeTriangles(
        const float* inVtx, const unsigned int* inTris, int nTris,
        const float* modelToClipMatrix = nullptr,
        MaskedOcclusionCulling::BackfaceWinding bfWinding = MaskedOcclusionCulling::BACKFACE_NONE,
        MaskedOcclusionCulling::ClipPlanes clipPlaneMask = MaskedOcclusionCulling::CLIP_PLANE_ALL,
        const MaskedOcclusionCulling::VertexLayout& vtxLayout =
            MaskedOcclusionCulling::VertexLayout(16, 4, 12));

    bool TestRect(float xmin, float ymin, float xmax, float ymax, float wmin) const;

    unsigned width() const { return width_; }
    unsigned height() const { return height_; }
    const std::vector<float>& depth() const { return openDepth_; }

private:
    void readDepth();

    MaskedOcclusionCulling* moc_;
    unsigned width_;
    unsigned height_;
    std::vector<float> openDepth_;
};
