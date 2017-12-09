// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once
#include<vector>
#include <unordered_map>
#include <set>
#include <dirent.h>
#include <iostream>

template<typename Key, typename Value>
using Hash_Map=std::unordered_map<Key, Value>;

struct Observation
{
  float x;
  float y;
  int id_feat;
};
using Observations=Hash_Map<int, Observation>;

struct Landmark
{
  double X;
  double Y;
  double Z;
  Observations obs;
};
using Landmarks=Hash_Map<int, Landmark>;

struct Document
{
  //SfM_Data _sfm_data;
  std::vector<std::string> m_image_list;
  Landmarks                m_control_points;

  bool loadData(const std::string & dImage)
  {
      DIR *dir;
      struct dirent *ptr;
      char base[1000];

      if ((dir=opendir(dImage.c_str())) == NULL)
      {
           perror("Open dir error...");
           exit(1);
      }

      while ((ptr=readdir(dir)) != NULL)
       {
          if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)
              continue;
          else if(ptr->d_type == 8)    ///file
                m_image_list.push_back(std::string(ptr->d_name));
          else if(ptr->d_type == 10)    ///link file
               m_image_list.push_back(std::string(ptr->d_name));
          else if(ptr->d_type == 4)    ///dir
          {
              memset(base,'\0',sizeof(base));
              strcpy(base,dImage.c_str());
              strcat(base,"/");
              strcat(base,ptr->d_name);
              loadData(base);
          }
       }
      closedir(dir);
      return true;
  }

  bool saveData(const std::string & sFileName)
  {
        //return Save(_sfm_data, sFileName, ESfM_Data(ALL));
        //保存的格式为:
        //GCPID Xs Ys Zs OnservationNumber
        //ObsID x y
        //...
        FILE* fgcps=nullptr;
        fgcps=fopen(sFileName.c_str(),"w+");
        int num = m_control_points.size();
        for(auto iterLandMark : m_control_points)
        {
            fprintf(fgcps,"%d %lf %lf %lf %d",iterLandMark.first,iterLandMark.second.X,
                    iterLandMark.second.Y,iterLandMark.second.Z,iterLandMark.second.obs.size());
            for(auto iterObs : iterLandMark.second.obs)
            {
                fprintf(fgcps,"%d %f %f",iterObs.first,iterObs.second.x,iterObs.second.y);
            }
            fprintf(fgcps,"\n");
        }
        fclose(fgcps);
        fgcps= nullptr;
        return true;
  }
};
