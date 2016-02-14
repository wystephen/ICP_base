#pragma once

#include <unsupported/Eigen/NumericalDiff>

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen:: Dynamic>
class FUNCTOR
{
public:
	typedef _Scalar Scalar;


	
	enum
	{
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	int m_inputs;
	int m_values;

	FUNCTOR():m_inputs(InputsAtCompileTime),m_values(ValuesAtCompileTime){}
	FUNCTOR(int inputs, int values) :m_inputs(inputs), m_values(values){}

	int inputs() const{ return m_inputs; }
	int values() const{ return m_values; }


	~FUNCTOR();
};

template <typename _Scalar, int NX, int NY>
FUNCTOR<_Scalar, NX, NY>::~FUNCTOR()
{
	m_inputs = 0;
}
