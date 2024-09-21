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

#include "Data.h"

namespace libRSF
{
  /** constructors */
  Data::Data()
  {
    this->Config = &GlobalDataConfig;
  }

  Data::Data(const std::string& Input)
  {
    this->Config = &GlobalDataConfig;
    this->constructFromString(Input);
  }

  Data::Data(DataType Type, double Timestamp)
  {
    this->Config = &GlobalDataConfig;
    this->constructEmpty(Type, Timestamp);
  }

  double Data::getTimestamp() const
  {
    return this->getValue(DataElement::Timestamp)(0);
  }

  Vector Data::getMean() const
  {
    return this->getValue(DataElement::Mean);
  }

  Matrix Data::getCovarianceMatrix() const
  {
    if (this->checkElement(DataElement::Covariance))
    {
      /** map vector to matrix */
      Vector CovVect = this->getValue(DataElement::Covariance);

      std::cout << "CovVect: " << CovVect.transpose() << std::endl;
    //   std::cout << "CovVect Size: " << CovVect.size() << std::endl;

    //   std::cout << static_cast<int>(sqrt(static_cast<double>(CovVect.size()))) << "\t"
    //             << static_cast<int>(sqrt(static_cast<double>(CovVect.size()))) << std::endl;
      
      MatrixRef<double, Dynamic, Dynamic> Cov(CovVect.data(),
                                              static_cast<int>(sqrt(static_cast<double>(CovVect.size()))),
                                              static_cast<int>(sqrt(static_cast<double>(CovVect.size()))));
      
      std::cout << "Cov1:\n" << Cov << std::endl;
    //   std::cout << "zzz" << std::endl;

    //   std::cout << "Rows1: " << static_cast<int>(sqrt(static_cast<double>(CovVect.size()))) << std::endl;
    //   std::cout << "Cols1: " << static_cast<int>(sqrt(static_cast<double>(CovVect.size()))) << std::endl;

    //   Eigen::MatrixXd DiagCov = CovVect.asDiagonal().toDenseMatrix();

    //   std::cout << "DiagCov:\n" << DiagCov << std::endl;

    //   MatrixRef<double, Dynamic, Dynamic> Cov(DiagCov.data(), DiagCov.rows(), DiagCov.cols());

    //   std::cout << "Cov:\n" << Cov << std::endl;



      return Cov;
    }
    if (this->checkElement(DataElement::CovarianceDiagonal))
    {
      return this->getCovarianceDiagonal().asDiagonal();
    }
    PRINT_ERROR("Data has no covariance element!");
    return {};
  }

  Vector Data::getCovarianceDiagonal() const
  {
    if (this->checkElement(DataElement::Covariance))
    {
      return this->getCovarianceMatrix().diagonal();
    }
    if (this->checkElement(DataElement::CovarianceDiagonal))
    {
      return this->getValue(DataElement::CovarianceDiagonal);
    }
    PRINT_ERROR("Data has no covariance element!");
    return {};
  }

  Vector Data::getStdDevDiagonal() const
  {
    if (this->checkElement(DataElement::Covariance))
    {
      std::cout << "111" << std::endl;
      return this->getCovarianceMatrix().diagonal().cwiseSqrt();
    }
    if (this->checkElement(DataElement::CovarianceDiagonal))
    {
      std::cout << "222" << std::endl;
      return this->getCovarianceDiagonal().cwiseSqrt();
    }
    PRINT_ERROR("Data has no covariance element!");
    return {};
  }

  double* Data::getMeanPointer()
  {
    return this->getDataPointer(DataElement::Mean);
  }

  double const* Data::getMeanPointerConst()
  {
    return this->getDataPointer(DataElement::Mean);
  }

  void Data::setMean(const Vector& Mean)
  {
    this->setValue(DataElement::Mean, Mean);
  }

  void Data::setTimestamp(const double Timestamp)
  {
    this->setValueScalar(DataElement::Timestamp, Timestamp);
  }

  void Data::setCovarianceDiagonal(const Vector& Cov)
  {
    // if (this->checkElement(DataElement::Covariance) && this->getValue(DataElement::Covariance).size() == 1)
    if (this->checkElement(DataElement::Covariance))
    {
      std::cout << "Covariance: " << Cov.transpose() << std::endl; 
      this->setValue(DataElement::Covariance, Cov);
    }
    else
    {
        // std::cout << "CovarianceDiagonal" << std::endl; 
      this->setValue(DataElement::CovarianceDiagonal, Cov);
    }
  }

  void Data::setCovariance(const Vector& Cov)
  {
    /** we expect a row-major vector representation of the actual matrix here */
    this->setValue(DataElement::Covariance, Cov);
  }

  void Data::setCovarianceMatrix(const Matrix& Cov)
  {
    /** map matrix to vector */
    const Eigen::Map<const Vector> CovVect(Cov.data(), Cov.size());

    /** we expect a row-major vector representation of the actual matrix here */
    this->setValue(DataElement::Covariance, CovVect);
  }


  void Data::setStdDevDiagonal(const Vector& StdDev)
  {
    this->setCovarianceDiagonal(StdDev.array().square());
  }

}
