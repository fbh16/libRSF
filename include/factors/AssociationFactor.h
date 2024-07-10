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
 * @file PriorFactor.h
 * @author Tim Pfeifer
 * @date 18.09.2018
 * @brief A factor that is able to represent prior knowledge or absolute measurements.
 * @copyright GNU Public License.
 *
 */

#ifndef ASSOCIATIONFACTOR_H
#define ASSOCIATIONFACTOR_H

#include "../Geometry.h"
#include "../VectorMath.h"
#include "BaseFactor.h"

namespace libRSF
{
  template <typename ErrorType, int Dim>
  class AssociationFactor : public BaseFactor<ErrorType, true, false, Dim>
  {
   public:
    /** construct factor and store measurement */
    AssociationFactor(ErrorType &Error, const Data &PriorMeasurement)
    {
      this->Error_ = Error;
      this->MeasurementVector_.resize(Dim);
      this->MeasurementVector_ = PriorMeasurement.getMean();  // 当前帧观测之一
    }

    /** geometric error model */
    template <typename T>
    VectorT<T, Dim> Evaluate(const T * const StatePointer) const
    {
      VectorRefConst<T, Dim> State(StatePointer);

      //   std::cout << "Evaluate:\n"
      //             << "MeasurementVector: " << this->MeasurementVector_.transpose()
      //             << "\nState:\n" << State << "\n" << std::endl;

      return State;  //* x
    }

    /** combine probabilistic and geometric model */
    template <typename T, typename... ParamsType>
    bool operator()(const T * const State, ParamsType... Params) const
    {
      return this->Error_.template weight<T>(this->Evaluate(State),
                                             Params...);
    }

    /** predict the next state for initialization */
    void predict(const std::vector<double *> &StatePointers) const
    {
      /** map pointer to vectors */
      VectorRef<double, Dim> State(StatePointers.at(0));
      State = this->MeasurementVector_;
    }
  };

  /** compile time mapping from factor type enum to corresponding factor class */
  template <typename ErrorType>
  struct FactorTypeTranslator<FactorType::Association2, ErrorType>
  {
    using Type = AssociationFactor<ErrorType, 2>;
  };
  template <typename ErrorType>
  struct FactorTypeTranslator<FactorType::Association3, ErrorType>
  {
    using Type = AssociationFactor<ErrorType, 3>;
  };
}  // namespace libRSF

#endif  // ASSOCIATIONFACTOR_H
