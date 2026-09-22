#include "OpenSpaceRasterizer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
float edge(float ax, float ay, float bx, float by, float px, float py) {
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}
}

OpenSpaceRasterizer::OpenSpaceRasterizer(unsigned width, unsigned height)
    : width_(width), height_(height), openDepth_(static_cast<std::size_t>(width) * height, 0.0f) {
    if (width == 0 || height == 0 || (width & 7u) != 0 || (height & 3u) != 0)
        throw std::invalid_argument("OpenSpaceRasterizer resolution must be width % 8 == 0 and height % 4 == 0");
}

void OpenSpaceRasterizer::ClearBuffer() {
    // Depth is MOC's reversed depth: z = 1 / w. Zero is the far/closed value;
    // rasterization keeps the maximum (nearest) open-space depth per pixel.
    std::fill(openDepth_.begin(), openDepth_.end(), 0.0f);
}

void OpenSpaceRasterizer::readVertex(const float* base, unsigned index,
                                     const float* matrix,
                                     const MaskedOcclusionCulling::VertexLayout& layout,
                                     Vertex& out) const {
    const char* p = reinterpret_cast<const char*>(base) + index * layout.mStride;
    const float x = *reinterpret_cast<const float*>(p);
    const float y = *reinterpret_cast<const float*>(p + layout.mOffsetY);
    const float inputZOrW = *reinterpret_cast<const float*>(p + layout.mOffsetW);

    if (!matrix) {
        // For already projected input, MOC's layout calls the third value W.
        // The z component is ignored and the depth is computed as 1 / W.
        out = {x, y, 0.0f, inputZOrW};
        return;
    }

    // For a model-to-clip transform, match MOC's convention: input is x,y,z
    // with an implicit input w of 1. The layout's third field is therefore Z.
    out = {
        matrix[0] * x + matrix[1] * y + matrix[2] * inputZOrW + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * inputZOrW + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * inputZOrW + matrix[11],
        matrix[12] * x + matrix[13] * y + matrix[14] * inputZOrW + matrix[15]
    };
}

void OpenSpaceRasterizer::rasterizeTriangle(const Vertex& a, const Vertex& b, const Vertex& c) {
    if (a.w <= 0.0f || b.w <= 0.0f || c.w <= 0.0f) return;

    const float ax = (a.x / a.w * 0.5f + 0.5f) * width_;
    const float ay = (a.y / a.w * 0.5f + 0.5f) * height_;
    const float bx = (b.x / b.w * 0.5f + 0.5f) * width_;
    const float by = (b.y / b.w * 0.5f + 0.5f) * height_;
    const float cx = (c.x / c.w * 0.5f + 0.5f) * width_;
    const float cy = (c.y / c.w * 0.5f + 0.5f) * height_;
    const float area = edge(ax, ay, bx, by, cx, cy);
    if (std::abs(area) < 1e-8f) return;

    const int minX = std::max(0, static_cast<int>(std::floor(std::min({ax, bx, cx}))));
    const int maxX = std::min(static_cast<int>(width_) - 1, static_cast<int>(std::ceil(std::max({ax, bx, cx}))));
    const int minY = std::max(0, static_cast<int>(std::floor(std::min({ay, by, cy}))));
    const int maxY = std::min(static_cast<int>(height_) - 1, static_cast<int>(std::ceil(std::max({ay, by, cy}))));

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const float px = x + 0.5f;
            const float py = y + 0.5f;
            const float e0 = edge(bx, by, cx, cy, px, py);
            const float e1 = edge(cx, cy, ax, ay, px, py);
            const float e2 = edge(ax, ay, bx, by, px, py);
            if (!((e0 >= 0 && e1 >= 0 && e2 >= 0) ||
                  (e0 <= 0 && e1 <= 0 && e2 <= 0))) {
                continue;
            }

            const float u = e0 / area;
            const float v = e1 / area;
            const float w = e2 / area;
            // Perspective-correct reciprocal-W interpolation. This is the same
            // reversed-depth convention used by MOC: larger values are nearer.
            const float depth = u / a.w + v / b.w + w / c.w;
            float& dst = openDepth_[static_cast<std::size_t>(y) * width_ + x];
            dst = std::max(dst, depth);
        }
    }
}

MaskedOcclusionCulling::CullingResult OpenSpaceRasterizer::RasterizeTriangles(
    const float* inVtx, const unsigned int* inTris, int nTris,
    const float* modelToClipMatrix,
    MaskedOcclusionCulling::BackfaceWinding bfWinding,
    MaskedOcclusionCulling::ClipPlanes,
    const MaskedOcclusionCulling::VertexLayout& layout) {
    if (!inVtx || !inTris || nTris <= 0)
        return MaskedOcclusionCulling::VIEW_CULLED;

    bool rasterized = false;
    for (int i = 0; i < nTris; ++i) {
        Vertex a{}, b{}, c{};
        readVertex(inVtx, inTris[i * 3], modelToClipMatrix, layout, a);
        readVertex(inVtx, inTris[i * 3 + 1], modelToClipMatrix, layout, b);
        readVertex(inVtx, inTris[i * 3 + 2], modelToClipMatrix, layout, c);
        if (a.w <= 0.0f || b.w <= 0.0f || c.w <= 0.0f) continue;

        const float area = (b.x / b.w - a.x / a.w) * (c.y / c.w - a.y / a.w) -
                           (b.y / b.w - a.y / a.w) * (c.x / c.w - a.x / a.w);
        const bool backface =
            (bfWinding == MaskedOcclusionCulling::BACKFACE_CW && area < 0.0f) ||
            (bfWinding == MaskedOcclusionCulling::BACKFACE_CCW && area > 0.0f);
        if (backface) continue;

        rasterizeTriangle(a, b, c);
        rasterized = true;
    }
    return rasterized ? MaskedOcclusionCulling::VISIBLE : MaskedOcclusionCulling::VIEW_CULLED;
}

bool OpenSpaceRasterizer::TestRect(float xmin, float ymin, float xmax, float ymax, float wmin) const {
    if (wmin <= 0.0f || xmax <= xmin || ymax <= ymin) return false;

    const int x0 = std::max(0, std::min(static_cast<int>(width_),
        static_cast<int>(std::floor((xmin + 1.0f) * width_ * 0.5f))));
    const int x1 = std::max(0, std::min(static_cast<int>(width_),
        static_cast<int>(std::ceil((xmax + 1.0f) * width_ * 0.5f))));
    const int y0 = std::max(0, std::min(static_cast<int>(height_),
        static_cast<int>(std::floor((ymin + 1.0f) * height_ * 0.5f))));
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
