# Add the coverage-oriented wrapper to the library target.
# It reuses MOC's existing SIMD triangle setup/rasterization through RenderTriangles
# and keeps the normal occlusion-buffer API unchanged.
set(MOC_OPEN_SPACE_FILES OpenSpaceRasterizer.cpp OpenSpaceRasterizer.h)
set(MOC_FILES ${MOC_FILES} ${MOC_OPEN_SPACE_FILES})
