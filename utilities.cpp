#include "utilities.h"





/*

     std::ofstream match0_log("D:/matchOrig_solarSlam.txt");
     match0_log<<matches.size()<<std::endl;
     for(auto &m:matches){
        match0_log<<m.getIndexInDescriptorA()<<" "<<m.getIndexInDescriptorB()<<std::endl;
     }
     match0_log.close();

      std::ofstream match1_log("D:/matchFund_solarSlam.txt");
      match1_log<<ggmatches.size()<<std::endl;
      for(auto &m:ggmatches){
         match1_log<<m.getIndexInDescriptorA()<<" "<<m.getIndexInDescriptorB()<<std::endl;
      }
      match1_log.close();

   std::ofstream oxMatrices("D:/matrices_solarSlam.txt");
   for(int i = 0; i < 3; ++i){
       for(int j = 0; j < 3; ++j){
           oxMatrices<<F(i,j)<<" ";
       }
       oxMatrices<<std::endl;
   }


       std::ofstream oxCloud("D:/cloud_solarSlam.txt");
       oxCloud<<gcloud.size()<<std::endl;
       for(auto & p: gcloud){
           oxCloud<<p->getX()<<" "<<p->getY()<<" "<<p->getZ()<<" "<<p->getReprojError()<<" "<<
                  p->m_visibility[0]<<" "<<p->m_visibility[1]<<std::endl;
       }
       oxCloud.close();




    std::cout<<"->K before proj: "<<std::endl;
    for(int ii = 0; ii < 3; ++ii){
        for(int jj = 0; jj < 3; ++jj){
            std::cout<<K(ii,jj)<<" ";
        }
        std::cout<<std::endl;
    }

    std::cout<<"->pose before proj: "<<std::endl;
        for(int ii = 0; ii < 4; ++ii){
            for(int jj = 0; jj < 4; ++jj){
                std::cout<<pose_current->m_poseTransform(ii,jj)<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl<<std::endl;


   std::ofstream oxMatches("D:/matchescurrent_solarSlam.txt");
   oxMatches<<new_matches_filtred.size()<<std::endl;
   for (int r = 0; r < new_matches_filtred.size(); ++r) {
     oxMatches << new_matches_filtred[r].getIndexInDescriptorA()<<" " <<new_matches_filtred[r].getIndexInDescriptorB()<<std::endl;
   }
   oxMatches.close();




    std::ofstream ox23d3d("D:/corr_solarSlam.txt");
    for (int tt = 0; tt < pt3d.size(); ++tt) {
        ox23d3d << pt3d[tt]->getX() << " " << pt3d[tt]->getY() << " " << pt3d[tt]->getZ() <<
                   " " << pt2d[tt]->getX() << " " << pt2d[tt]->getY() << std::endl;
    }
    ox23d3d.close();



   std::ofstream oxKp("D:/kpcurrent_solarSlam.txt");
   oxKp<<keypoints3.size()<<std::endl;
   for (int r = 0; r < keypoints3.size(); ++r) {
     oxKp << keypoints3[r]->getX()<<" " <<keypoints3[r]->getY()<<std::endl;
   }
   oxKp.close();


/*
std::cout<<"->F: "<<std::endl;
for(int ii = 0; ii < 3; ++ii){
    for(int jj = 0; jj < 3; ++jj){
        std::cout<<F(ii,jj)<<" ";
    }
    std::cout<<std::endl;
}

std::cout<<"->K: "<<std::endl;
for(int ii = 0; ii < 3; ++ii){
    for(int jj = 0; jj < 3; ++jj){
        std::cout<<K(ii,jj)<<" ";
    }
    std::cout<<std::endl;
}

std::cout<<" Poses size: "<<poses.size()<<std::endl;
for(int k = 0; k <poses.size(); ++k){

    std::cout<<"--pose: "<<k<<std::endl;
    for(int ii = 0; ii < 4; ++ii){
        for(int jj = 0; jj < 4; ++jj){
            std::cout<<poses[k]->m_poseTransform(ii,jj)<<" ";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl<<std::endl;
}
*/

