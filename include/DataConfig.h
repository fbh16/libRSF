/***************************************************************************
 * libRSF - A Robust Sensor Fusion Library
 *
 * Copyright (C) 2023 Chair of Automation Technology / TU Chemnitz
 * For more information see https://www.tu-chemnitz.de/etit/proaut/libRSF
 *
 * libRSF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libRSF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with libRSF.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tim Pfeifer (tim.pfeifer@etit.tu-chemnitz.de)
 ***************************************************************************/

/**
 * @file DataConfig.h
 * @author Tim Pfeifer
 * @date 09.03.2021
 * @brief This holds the configuration for different types of structured data.
 * @copyright GNU Public License.
 *
 */

#ifndef DATACONFIG_H
#define DATACONFIG_H

#include <map>
#include <vector>
#include <string>
#include <iostream>

namespace libRSF
{
  template<typename TypeEnum, typename ElementEnum>
  class DataConfig
  {
    public:

      /** define types that store the configuration */
      using ConfigType = std::vector<std::pair<ElementEnum, int>>;
      using InitType = struct
      {
        std::string Name;
        TypeEnum Type;
        ConfigType Elements;
      };
      using InitVect = std::vector<InitType>;

      /** disable the default constructor construction */
      DataConfig() = delete;

      /** enforce advanced initialization */
      explicit DataConfig(InitVect InitialConfig)
      {
        for (InitType &Init : InitialConfig)
        {
          TypeMap_.emplace(Init.Type, Init.Elements); //Init.Elements每个参数的初始维度
          NameTypeMap_.emplace(Init.Name, Init.Type);
          TypeNameMap_.emplace(Init.Type, Init.Name);
        }
      }

      /** destruction */
      virtual ~DataConfig() = default;

      /** query string */
      std::string getName(TypeEnum Type) const
      {
        return TypeNameMap_.at(Type);
      }

      TypeEnum getType(std::string Name) const
      {
        // std::cout << "123  " << NameTypeMap_.at(Name) << std::endl;
        /**
         * 返回因子类型对应的enum值
        */
        return NameTypeMap_.at(Name);
      }
      
      
      /** check type or string */
      [[nodiscard]] bool checkName(std::string Name) const
      {
        // std::cout << "NameTypeMap Size: " << NameTypeMap_.size() << std::endl; 
        // for (auto it = NameTypeMap_.begin(); it != NameTypeMap_.end(); it++) {
        //     std::cout << it->first << "\t" << it->second << std::endl;
        // }
        // std::cout << std::endl;
        
        /**
         * NameTypeMap_是包含53组键值对的map，
         * 键是因子名称，值是因子类型，e.g. imu->42, range2->22, point3->2...
         */ 
        return (NameTypeMap_.count(Name) > 0);
      }

      bool checkType(TypeEnum Type) const
      {
        // for (auto it = TypeNameMap_.begin(); it != TypeNameMap_.end(); it++) {
        //     std::cout << it->first << "\t" << it->second << std::endl;
        // }
        // std::cout << std::endl;
        /**
         * TypeNameMap_是包含53组键值对的map，
         * 键是因子类型，值是因子名称. e.g. 22->range2
        */
        return (TypeNameMap_.count(Type) > 0);
      }

      /** query config */
      const ConfigType &getConfig(std::string ID) const
      {
        return TypeMap_.at(NameTypeMap_.at(ID));
      }

      const ConfigType &getConfig(TypeEnum Type) const
      {
        return TypeMap_.at(Type);
      }

    private:
      std::map<std::string, TypeEnum> NameTypeMap_;
      std::map<TypeEnum, std::string> TypeNameMap_;
      std::map<TypeEnum, ConfigType> TypeMap_; // ConfigType是初始化向量的尺寸，e.g. 
        /**
         * "odom2diff", DataType::Odom2Diff,
        {
            {DataElement::Timestamp, 1},
            {DataElement::Mean, 3},
            {DataElement::WheelBase, 1},
            {DataElement::CovarianceDiagonal , 3}
        }
        * 是1,3,1,3.所以就将ConfigType初始化为同样大小的向量Vector
        */
  };
}

#endif // DATACONFIG_H
