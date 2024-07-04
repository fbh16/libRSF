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
 * @file FactorGraph.h
 * @author Tim Pfeifer
 * @date 06.06.2018
 * @brief Main class that contains the optimization problem and several interfaces and method to interact with it.
 * @copyright GNU Public License.
 *
 */

#ifndef FACTORGRAPH_H
#define FACTORGRAPH_H

#include <ceres/ceres.h>
#include <ceres/normal_prior.h>

#include <thread>

#include "CalculateCovariance.h"
#include "DataSet.h"
#include "FactorGraphSampling.h"
#include "FactorGraphStructure.h"
#include "FactorIDSet.h"
#include "FileAccess.h"
#include "LocalParametrization.h"
#include "Marginalization.h"
#include "SensorDataSet.h"
#include "StateDataSet.h"
#include "TimeMeasurement.h"
#include "Types.h"
#include "error_models/DynamicCovarianceEstimation.h"
#include "error_models/ErrorModel.h"
#include "error_models/Gaussian.h"
#include "error_models/LossFunction.h"
#include "error_models/MaxMixture.h"
#include "error_models/MaxSumMixture.h"
#include "error_models/SumMixture.h"
#include "error_models/SwitchableConstraints.h"
#include "factors/BaseFactor.h"
#include "factors/BearingRangeFactor.h"
#include "factors/BetweenPose2Factor.h"
#include "factors/BetweenPose3Factor.h"
#include "factors/BetweenQuaternionFactor.h"
#include "factors/BetweenValueFactor.h"
#include "factors/ConstantDriftFactor.h"
#include "factors/ConstantPose2Factor.h"
#include "factors/ConstantQuaternionFactor.h"
#include "factors/ConstantValueFactor.h"
#include "factors/IMUFactor.h"
#include "factors/IMUPreintegrationFactor.h"
#include "factors/MarginalPrior.h"
#include "factors/OdometryFactor2D.h"
#include "factors/OdometryFactor2DDifferential.h"
#include "factors/OdometryFactor3D.h"
#include "factors/PointRegistrationFactor.h"
#include "factors/PressureDifferenceFactor.h"
#include "factors/PriorFactor.h"
#include "factors/PseudorangeFactor.h"
#include "factors/RangeFactor.h"
#include "factors/RangeToPointFactor.h"
#include "factors/TrackingDetectionFactor.h"
#include "factors/TrackingFactor.h"

namespace libRSF
{
  class StateList
  {
   public:
    StateList() = default;
    virtual ~StateList() = default;

    void add(const string &Type, double Timestamp, int Number = 0);
    void add(const StateID &);
    void clear();

    std::vector<StateID> List_;
  };

  class FactorGraph
  {
   public:
    /** Default constructor */
    FactorGraph();
    /** Default destructor */
    virtual ~FactorGraph() = default;

    /** access to single states */
    void addState(const string &Name, DataType Type, double Timestamp);
    void addState(const string &Name, Data &Element);

    /** add states only if they doesn't exist */
    void addStateWithCheck(const string &Name, DataType Type, double Timestamp);

    /** access to complete state data */
    StateDataSet &getStateData();

    /** toggle error models of a specific factor */
    void enableErrorModel(FactorType CurrentFactorType);
    void disableErrorModel(FactorType CurrentFactorType);

    /** toggle all error models */
    void enableErrorModels();
    void disableErrorModels();

    /** replace error models which are given as pointer of vectors */
    template <typename ErrorType>
    void setNewErrorModel(const std::vector<ErrorModelBase *> &ErrorModels, const ErrorType &NoiseModel)
    {
      for (const auto &ErrorModel : ErrorModels)
      {
        *static_cast<ErrorType *>(ErrorModel) = NoiseModel;
      }
    }
    /** replace error model of all factors of a specific type */
    template <typename ErrorType>
    void setNewErrorModel(FactorType CurrentFactorType, const ErrorType &NoiseModel)
    {
      /** find error model pointers */
      std::vector<ErrorModelBase *> ErrorModels;
      Structure_.getErrorModels(CurrentFactorType, ErrorModels);

      /** replace */
      setNewErrorModel(ErrorModels, NoiseModel);
    }

    /** replace error model of a specific factor */
    template <typename ErrorType>
    void setNewErrorModel(FactorType CurrentFactorType, double TimeStamp, int Number, const ErrorType &NoiseModel)
    {
      /** find the right error model pointer */
      ErrorModelBase * ModelPointer;
      FactorID ID(CurrentFactorType, TimeStamp, Number);
      Structure_.getErrorModel(ID, ModelPointer);

      /** replace */
      std::vector<ErrorModelBase *> ModelPointers = {ModelPointer};
      setNewErrorModel(ModelPointers, NoiseModel);
    }

    /** add factors with variable number of states - first level*/
    template <FactorType CurrentFactorType, typename... FactorParameters>
    void addFactor(const StateID &ID1,
                   FactorParameters... Params)  // ID2,Noise
    {
      //** compile time error for IMU pre-integration */
      static_assert(CurrentFactorType != FactorType::IMUPretintegration, "Do not use the default factor interface for IMU pre-integration factor! Use addIMUPreintegrationFactor() instead!");
    //   std::cout << "  the first factor" << std::endl;
      StateList List;
      List.add(ID1);                                  //*连接因子与第一个状态变量
      addFactor<CurrentFactorType>(List, Params...);  // ID2,Noise
    }

    //** in-between level */
    template <FactorType CurrentFactorType, typename... FactorParameters>
    void addFactor(StateList List,
                   const StateID &ID,
                   FactorParameters... Params)  // Noise
    {
    //   std::cout << "  the second factor" << std::endl;
      /** move state ID into a state list */
      List.add(ID);  //*连接因子与第二个状态变量
      /** evaluate parameter pack again */
      addFactor<CurrentFactorType>(List, Params...);
    }

    //** last level without measurement*/
    template <FactorType CurrentFactorType, typename ErrorType>
    void addFactor(StateList List,
                   ErrorType &NoiseModel,
                   ceres::LossFunction * RobustLoss,
                   const bool DoPrediction = true)
    {
      //   std::cout << "without measurement" << std::endl;
      addFactorBase_<CurrentFactorType>(List, NoiseModel, Data(DataType::Value1, 0.0), RobustLoss, DoPrediction);
    }
    template <FactorType CurrentFactorType, typename ErrorType>
    void addFactor(StateList List,
                   ErrorType &NoiseModel,
                   const bool DoPrediction = true)
    {
    //   std::cout << "  without measurement" << std::endl;
      addFactorBase_<CurrentFactorType>(List, NoiseModel, Data(DataType::Value1, 0.0), nullptr, DoPrediction);
    }

    //** last level with measurement */
    template <FactorType CurrentFactorType, typename ErrorType>
    void addFactor(StateList List,
                   const Data &Measurement,
                   ErrorType &NoiseModel,
                   ceres::LossFunction * RobustLoss,
                   const bool DoPrediction = true)
    {
      //   std::cout << "with measurement" << std::endl;
      addFactorBase_<CurrentFactorType>(List, NoiseModel, Measurement, RobustLoss, DoPrediction);
    }
    template <FactorType CurrentFactorType, typename ErrorType>
    void addFactor(StateList List,
                   const Data &Measurement,
                   ErrorType &NoiseModel,
                   const bool DoPrediction = true)
    {
    //   std::cout << "  with measurement" << std::endl;
      addFactorBase_<CurrentFactorType>(List, NoiseModel, Measurement, nullptr, DoPrediction);
    }

    /** special case for IMU pre-integration */
    void addIMUPreintegrationFactor(StateList List, const PreintegratedIMUResult &IMUState);

    /** for sliding window */
    void removeFactor(FactorType CurrentFactorType, double Timestamp);
    void removeFactorsOutsideWindow(FactorType CurrentFactorType, double TimeWindow, double CurrentTime);
    void removeAllFactorsOutsideWindow(double TimeWindow, double CurrentTime);

    /** solve problem */
    void solve();
    void solve(ceres::Solver::Options Options);

    /** compute covariances */
    bool computeCovarianceSigmaPoints(const string &Name, double Timestamp, int StateNumber = 0);
    bool computeCovariance(const string &Name, double Timestamp);
    bool computeCovariance(const string &Name);
    bool computeCovariance(const string &Name, double Timestamp, int Number);

    /** marginalize factors */
    bool marginalizeState(const string &Name, double Timestamp, int Number = 0);
    bool marginalizeStates(std::vector<StateID> States, double Inflation = 1.0);
    bool marginalizeAllStatesOutsideWindow(double TimeWindow, double CurrentTime, double Inflation = 1.0);

    /** sample output state */
    void sampleCost1D(const string &StateName,
                      double Timestamp,
                      int Number,
                      int PointCount,
                      double Range,
                      StateDataSet &Result,
                      const bool OptimizeOther = false);

    void sampleCost2D(const string &StateName,
                      double Timestamp,
                      int Number,
                      int PointCount,
                      double Range,
                      StateDataSet &Result);

    void sampleCost3D(const string &StateName,
                      double Timestamp,
                      int Number,
                      int PointCount,
                      double Range,
                      StateDataSet &Result);

    /** remove old states*/
    void removeState(const string &Name, double Timestamp);
    void removeState(const string &Name, double Timestamp, int Number);
    void removeStatesOutsideWindow(const string &Name, double TimeWindow, double CurrentTime);
    void removeAllStatesOutsideWindow(double TimeWindow, double CurrentTime);

    /** handle constant states */
    void setConstant(const string &Name, double Timestamp);
    void setVariable(const string &Name, double Timestamp);
    void setVariable(const string &Name);

    void setSubsetConstant(const string &Name, double Timestamp, int Number, const std::vector<int> &ConstantIndex);

    void setConstantOutsideWindow(const string &Name, double TimeWindow, double CurrentTime);
    void setVariableInsideWindow(const string &Name, double TimeWindow, double CurrentTime);
    void setAllConstantOutsideWindow(double TimeWindow, double CurrentTime);
    void setAllVariableInsideWindow(double TimeWindow, double CurrentTime);
    void setAllVariable();

    /** handle bound */
    void setUpperBound(const string &Name, double Timestamp, int StateNumber, const Vector &Bound);
    void setLowerBound(const string &Name, double Timestamp, int StateNumber, const Vector &Bound);

    /** get information about the structure */
    void getFactorsOfState(const string &Name, double Timestamp, int Number, std::vector<FactorID> &Factors) const;
    int countFactorsOfType(FactorType CurrentFactorType) const;

    /** compute overall error error models */
    double getCost();

    /** compute raw errors without error models */
    void computeUnweightedError(FactorType CurrentFactorType, std::vector<double> &ErrorData);
    void computeUnweightedErrorMatrix(FactorType CurrentFactorType, Matrix &ErrorMatrix);
    void computeUnweightedError(FactorType CurrentFactorType, const string &Name, StateDataSet &ErrorData);
    void computeUnweightedError(FactorType CurrentFactorType, double Time, int Number, Vector &Error);

    /** access solver options */
    void setSolverOptions(const ceres::Solver::Options &Options);
    ceres::Solver::Options getSolverOptions() const;

    /** access solver summary */
    void printReport() const;
    ceres::Solver::Summary getSolverSummary() const;
    int getSolverIterationsAndReset();
    double getSolverDurationAndReset();
    double getMarginalDurationAndReset();
    double getCovarianceDurationAndReset();
    ceres::Problem &getProblem() { return Graph_; }
    // double &getCovarianceDuration() { return CovarianceDuration_; }
    FactorGraphStructure &getStructure() {return Structure_;}
    
   private:
    /** helper function to create variadic templated cost functions */
    template <int NoiseModelOutputDim, typename FactorClass, int... FactorStateDims, int... ErrorModelStateDims>
    auto makeAutoDiffCostFunction_(FactorClass * Factor, std::integer_sequence<int, FactorStateDims...>, std::integer_sequence<int, ErrorModelStateDims...>)
    {
      return new ceres::AutoDiffCostFunction<FactorClass, NoiseModelOutputDim, FactorStateDims..., ErrorModelStateDims...>(Factor);
    }

    /** add and remove factors */
    template <typename ErrorType, typename FactorClass, typename... FactorParameters>
    void addFactorGeneric_(ErrorType &NoiseModel,
                           const std::vector<StateID> &StateList,
                           const FactorType FactorTypeEnum,
                           ceres::LossFunction * RobustLoss,
                           const bool DoPrediction,
                           const double Timestamp,
                           FactorParameters... Params)
    {
      /** build list of states */
      std::vector<double *> StatePointers;

    //   std::cout << "-- " << StateList[0].ID << " STATE: " << StateList.size() << " --" << std::endl;
    //   std::cout << "STATE DATA: \n";
    //   int i = 1;
      for (const StateID &State : StateList)
      {
        StatePointers.emplace_back(
            StateData_.getElement(State.ID, State.Timestamp, State.Number).getMeanPointer());

    //     std::cout << " No." << i << ": " << StateData_.getElement(State.ID, State.Timestamp, State.Number).getMean().x() << " "
    //               << StateData_.getElement(State.ID, State.Timestamp, State.Number).getMean().y() << std::endl;
    //     i++;
      }
    //   std::cout << "\n";

      /** create factor object */
      auto * Factor = new FactorClass(NoiseModel, Params...);

      /** use the factor to predict */
      if (DoPrediction)
      {
        Factor->predict(StatePointers);
      }

    //   std::cout << "STATE DATA2: \n";
    //   int j = 1;
    //   for (const StateID &State : StateList)
    //   {
    //     std::cout << " No." << j << ": " << StateData_.getElement(State.ID, State.Timestamp, State.Number).getMean().x() << " "
    //               << StateData_.getElement(State.ID, State.Timestamp, State.Number).getMean().y() << std::endl;
    //     j++;
    //   }
    //   std::cout << "\n";

      /** wrap it in ceres cost function
       ** 创建一个自动微分的代价函数对象，用于在Ceres Solver中进行优化问题的求解。
       ** Factor中的模板方法即为CostFunctor
       */
      auto CostFunction = makeAutoDiffCostFunction_<ErrorType::OutputDim, FactorClass>(Factor,
                                                                                       typename FactorClass::StateDims{},
                                                                                       typename ErrorType::StateDims{});
      /** add it to the estimation problem
       ** 代价函数CostFunction携带有关所期望的因子维度信息,该函数检查这些维度是否正确,若检测出维度不匹配问题则程序终止。
       ** RobustLoss可以为nullptr，在这种情况下，该项的代价仅为残差的平方范数.
       */
      ceres::ResidualBlockId CurrentCeresFactorID =
          Graph_.AddResidualBlock(CostFunction,
                                  RobustLoss,
                                  StatePointers);

      /** store state types */
      std::vector<DataType> StateTypes;
      for (const StateID &State : StateList)
      {
        StateTypes.emplace_back(
            StateData_.getElement(State.ID, State.Timestamp, State.Number).getType());
      }
      //   for (const StateID &State : StateList)
      //   {
      //     std::cout << "State Data2: \n" <<
      //         StateData_.getElement(State.ID, State.Timestamp, State.Number).getMean() << std::endl;
      //   }

      /** store the graphs structure */
      Structure_.addFactor<ErrorType>(FactorTypeEnum,
                                      Timestamp,
                                      CurrentCeresFactorID,
                                      Factor->getErrorModel(),
                                      StateList,
                                      StatePointers,
                                      StateTypes);
    }

    template <FactorType CurrentFactorType, typename ErrorType>
    void addFactorBase_(StateList &States, ErrorType &NoiseModel, const Data &Measurement, ceres::LossFunction * RobustLoss, const bool DoPrediction)
    {
      /**
       ** 如果是通过无观测方式添加的因子(e.g. ConstVal)，Measurement为0；
       ** 否则，Measurement为被添加的观测信息(e.g. Range2)；
       */

      /** get index timestamp */
      const double TimestampFirst = States.List_.front().Timestamp;

      /** translate the factor type enum to the class that should be added to the graph */
      using FactorClassType = typename FactorTypeTranslator<CurrentFactorType, ErrorType>::Type;  // 待添加的因子类

      /** decide at compile time which parameters are required */
      if constexpr (static_cast<bool>(FactorClassType::HasDeltaTime) && static_cast<bool>(FactorClassType::HasMeasurement))
      {
        //* in-between类型因子，且有观测信息输入
        /** calculate delta time */
        const double DeltaTime = States.List_.back().Timestamp - TimestampFirst;
        // std::cout << "." << std::endl;
        addFactorGeneric_<ErrorType, FactorClassType>(NoiseModel,
                                                      States.List_,
                                                      CurrentFactorType,
                                                      RobustLoss,
                                                      DoPrediction,
                                                      TimestampFirst,
                                                      Measurement,
                                                      DeltaTime);
      }
      else if constexpr (!static_cast<bool>(FactorClassType::HasDeltaTime) && static_cast<bool>(FactorClassType::HasMeasurement))
      {
        // std::cout << ".." << std::endl;
        //* 非in-between类型因子，且有观测信息输入. e.g. Range2
        addFactorGeneric_<ErrorType, FactorClassType>(NoiseModel,
                                                      States.List_,
                                                      CurrentFactorType,
                                                      RobustLoss,
                                                      DoPrediction,
                                                      TimestampFirst,
                                                      Measurement);
      }
      else if constexpr (static_cast<bool>(FactorClassType::HasDeltaTime) && !static_cast<bool>(FactorClassType::HasMeasurement))
      {
        // std::cout << "..." << std::endl;
        /** calculate delta time */
        const double DeltaTime = States.List_.back().Timestamp - TimestampFirst;

        addFactorGeneric_<ErrorType, FactorClassType>(NoiseModel,
                                                      States.List_,
                                                      CurrentFactorType,
                                                      RobustLoss,
                                                      DoPrediction,
                                                      TimestampFirst,
                                                      DeltaTime);
      }
      else if constexpr (!static_cast<bool>(FactorClassType::HasDeltaTime) && !static_cast<bool>(FactorClassType::HasMeasurement))
      {
        // std::cout << "...." << std::endl;
        //* in-between类型因子，且无观测信息输入. e.g. ConstVal2
        addFactorGeneric_<ErrorType, FactorClassType>(NoiseModel,
                                                      States.List_,
                                                      CurrentFactorType,
                                                      RobustLoss,
                                                      DoPrediction,
                                                      TimestampFirst);
      }
    }

    /** default settings for new ceres::Problems, especially enable_fast_removal = true */
    const ceres::Problem::Options DefaultProblemOptions_ = {
        ceres::Ownership::TAKE_OWNERSHIP,  // cost_function_ownership
        ceres::Ownership::TAKE_OWNERSHIP,  // loss_function_ownership
        ceres::Ownership::TAKE_OWNERSHIP,  // local_parameterization_ownership
        true,                              // enable_fast_removal
        false,                             // disable_all_safety_checks
        nullptr,                           // context
        nullptr                            // evaluation_callback
    };

    ceres::Problem Graph_;
    ceres::Solver::Summary Report_;
    ceres::Solver::Options SolverOptions_;

    StateDataSet StateData_;         /**< holds all state variables */
    FactorGraphStructure Structure_; /**< represents the structure of variables and factors */

    /** store information about the past computational load */
    double SolverDuration_;
    int SolverIterations_;
    double MarginalizationDuration_;
    double CovarianceDuration_;
  };
}  // namespace libRSF

#endif  // FACTORGRAPH_H
