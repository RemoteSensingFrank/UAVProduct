//
// Created by wuwei on 17-7-30.
//

#ifndef UAVPRODUCT_UAVAUXILIARY_H
#define UAVPRODUCT_UAVAUXILIARY_H

#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "third_party/vectorGraphics/svgDrawer.hpp"

/// Export an adjacency matrix as a SVG file
void AdjacencyMatrixToSVG
        (
                const size_t NbImages,
                const openMVG::Pair_Set & corresponding_indexes,
                const std::string & sOutName
        );

#endif //UAVPRODUCT_UAVAUXILIARY_H
