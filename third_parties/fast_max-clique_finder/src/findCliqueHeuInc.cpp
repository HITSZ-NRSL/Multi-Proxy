// This file is modified from the original file at:
// https://github.com/MIT-SPARK/Kimera-RPGO/blob/master/include/KimeraRPGO/max_clique_finder/findCliqueHeu.cpp
#include "findClique.h"
#include <cstddef>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
namespace FMC {
int maxCliqueHeuIncremental(CGraphIO& gio,
                            size_t num_new_lc,
                            size_t prev_maxclique_size,
                            vector<int> & max_clique_data) {
  vector<int>* p_v_i_Vertices = gio.GetVerticesPtr();
  vector<int>* p_v_i_Edges = gio.GetEdgesPtr();
  // srand(time(NULL));

  int maxDegree = gio.GetMaximumVertexDegree();
  // TODO initialize maxClq with the best so far from prev steps
  int maxClq = prev_maxclique_size, u, icc;
  vector<int> neighboors_of_candidate_vertex;
  vector<int> v_i_S1;
  neighboors_of_candidate_vertex.resize(maxDegree + 1, -1);
  v_i_S1.resize(maxDegree + 1, -1);

  int candidate_neighboors, iPos1, iCount, iCount1;

  int notComputed = 0;


  // TODO tricky indexing ...
  for (size_t iCandidateVertex = p_v_i_Vertices->size() - num_new_lc - 1;
       iCandidateVertex < p_v_i_Vertices->size() - 1;
       iCandidateVertex++) {
    // Pruning 1
    if (maxClq > ((*p_v_i_Vertices)[iCandidateVertex + 1] -
                  (*p_v_i_Vertices)[iCandidateVertex])) {
      notComputed++;
      continue;
    }

    candidate_neighboors = 0;
    neighboors_of_candidate_vertex[candidate_neighboors++] = iCandidateVertex;
    int z, imdv = iCandidateVertex,
           imd = (*p_v_i_Vertices)[iCandidateVertex + 1] -
                 (*p_v_i_Vertices)[iCandidateVertex];

    int iLoopCount = (*p_v_i_Vertices)[iCandidateVertex + 1];
    for (int j = (*p_v_i_Vertices)[iCandidateVertex]; j < iLoopCount; j++) {
      // Pruning 3
      if (maxClq <= ((*p_v_i_Vertices)[(*p_v_i_Edges)[j] + 1] -
                     (*p_v_i_Vertices)[(*p_v_i_Edges)[j]]))
        neighboors_of_candidate_vertex[candidate_neighboors++] = (*p_v_i_Edges)[j];
    }

    icc = 0;

    while (candidate_neighboors > 0 && candidate_neighboors + icc > maxClq) {
      int imdv1 = -1, imd1 = -1;

      icc++;

      // generate a random number x from 0 to candidate_neighboors -1
      // aasign that imdv = x and imd = d(x)
      // imdv = neighboors_of_candidate_vertex[rand() % candidate_neighboors];
      imdv = neighboors_of_candidate_vertex[candidate_neighboors - 1];
      imd = (*p_v_i_Vertices)[imdv + 1] - (*p_v_i_Vertices)[imdv];

      iPos1 = 0;

      for (int j = 0; j < candidate_neighboors; j++) {

        iLoopCount = (*p_v_i_Vertices)[imdv + 1];
        for (int k = (*p_v_i_Vertices)[imdv]; k < iLoopCount; k++) {

          if (neighboors_of_candidate_vertex[j] == (*p_v_i_Edges)[k] &&
              maxClq <= ((*p_v_i_Vertices)[(*p_v_i_Edges)[k] + 1] -
                         (*p_v_i_Vertices)[(*p_v_i_Edges)[k]])) {
            v_i_S1[iPos1++] = neighboors_of_candidate_vertex[j];  // calculate the max degree vertex here

            break;
          }
        }
      }

      for (int j = 0; j < candidate_neighboors - 1; j++) {
        if (j < iPos1) {
          neighboors_of_candidate_vertex[j] = v_i_S1[j];
        }
        else {
          neighboors_of_candidate_vertex[j] = -1;
        }
      }

//      if (iPos1 < candidate_neighboors - 1) {
//        std::cout << "Error: iPos1 < candidate_neighboors - 1, "
//                  << "iPos1: " << iPos1
//                  << ", candidate_neighboors: " << candidate_neighboors
//                  << ", check pos: " << imdv
//                  << std::endl;
//      }

      candidate_neighboors = iPos1;

      imdv = imdv1;
      imd = imd1;
    }

    if (maxClq < icc) {
      max_clique_data = neighboors_of_candidate_vertex;
      maxClq = icc;
    }
  }

  return maxClq;
}
}