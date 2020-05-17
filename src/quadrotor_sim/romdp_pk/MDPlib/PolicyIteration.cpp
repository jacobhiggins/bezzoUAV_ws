#include <eigen3/Eigen/Dense>
#include "PolicyIteration.h"
#include "model.h"
#include <iostream>
#include "header/newType.h"

namespace MDP
{
	PolicyIteration::PolicyIteration(Model &model)
	{
		eval_type = 0; // default
		max_iter = 1000; // default
		iter = 0;

		N = model.get_state_num();
		M = model.get_action_num();
		discount = model.getDiscount();

		value = Eigen::MatrixXd::Zero(N,1);
		policy = Eigen::MatrixXd::Zero(N, 1);
		reward = PR = Eigen::MatrixXd::Zero(N, M);

		//pv.value = value;
		//pv.policy = policy;
	}
	PolicyIteration::~PolicyIteration()
	{

	}

	void PolicyIteration::reset()
	{
		//eval_type = 0; // default
		//max_iter = 1000; // default
		iter = 0;
		//discount = model_.getDiscount();
		
		//N = model_.getNumOfState();
		//M = model_.getNumOfAction();

		value = Eigen::MatrixXd::Zero(N,1);
		policy = Eigen::MatrixXd::Zero(N, 1);
		reward = PR = Eigen::MatrixXd::Zero(N, M);

	}

	Policy_and_Value PolicyIteration::eval()
	{
		computePR();

		// initialize policy
		bellman_operator();
		Eigen::MatrixXd policy_next = policy;

		// loop
		bool is_done = false;
		while (!is_done)
		{
			iter += 1;

			if (eval_type == 0)
			{

				eval_policy_matrix();
			}
			else
			{
				eval_policy_iterative();
			}
			// new policy
			bellman_operator();
		        //std::cout<<"iter= "<<iter<<std::endl;
			//std::cout<<"value= "<<value(0)<<std::endl;
			//std::cout<<"policy= "<<policy.sum() + 100<<std::endl;
			// termination check
			int diff_policy = 0;
			for (int i = 0; i < N; i++)
			{
				if (policy_next(i) != policy(i)) diff_policy++;
			}
			//if (( (policy_next - policy).sum() == 0) || iter == max_iter) is_done = true; // end loop
			if (( diff_policy == 0) || iter == max_iter) is_done = true; // end loop
			else  policy_next = policy; // continue
		}

		Policy_and_Value pv;  // short for policy and value
		pv.value = value;
		pv.policy = policy;
		// add one to make sense
		pv.policy.array() += 1;
		return pv;
	}
	// set transition matrix
	void PolicyIteration::set_transition_matrix(Eigen::MatrixXd *trans_3d_)
	{
		trans_3d = trans_3d_;
		//std::cout<<trans_3d_[1]<<std::endl;
		//std::cout<<trans_3d[0]<<std::endl;std::cout<<"\n"<<std::endl;
		//std::cout<<trans_3d[1]<<std::endl;std::cout<<"\n"<<std::endl;
		//std::cout<<trans_3d[2]<<std::endl;std::cout<<"\n"<<std::endl;
		//std::cout<<trans_3d[3]<<std::endl;std::cout<<"\n"<<std::endl;

	}

	// set reward matrix
	void PolicyIteration::set_reward_matrix(const Eigen::MatrixXd reward_)
	{
		reward = reward_;
		//std::cout<<reward<<std::endl;std::cout<<"\n"<<std::endl;
	}

	void PolicyIteration::update_transition_matrix(const int c1_, const int c2_, const int c3_)
	{
		double p1 = 1.0 / 3.0;
		int c1 = c1_, c2 = c2_, c3 = c3_;
		for (int act = 0; act < M; act++)
		{
				for (int j = 0; j < N; j++)
				{
					//std::cout<<trans_3d[act](c1, j)<<std::endl;
					trans_3d[act](c1, j) = p1 * ( trans_3d[act](c1, j) + trans_3d[act](c2, j) + trans_3d[act](c3, j) ) ;
					trans_3d[act](c3, j) = trans_3d[act](c1, j);
					trans_3d[act](c2, j) = trans_3d[act](c1, j);
				}
				//std::cout<<trans_3d[act].sum()<<std::endl;
		}
	}

	void PolicyIteration::update_reward_matrix(const int c1_, const int c2_, const int c3_)
	{
		double p1 = 1.0 / 3.0;
		int c1 = c1_, c2 = c2_, c3 = c3_;
		for (int act = 0; act < M; act++)
		{
			reward(c1, act) = p1 * ( reward(c1, act) + reward(c2, act) + reward(c3, act) ) ;
			reward(c3, act) = reward(c1, act);
			reward(c2, act) = reward(c1, act);
		}

		PR = reward;

	}

	// set eval type
	void PolicyIteration::set_eval_type(const int eval_type_ )  
	{
		eval_type = eval_type_;
	}
	const int PolicyIteration::get_eval_type()
	{
		return eval_type;
	}

	// set max iteration
	void PolicyIteration::set_max_iter(const int max_iter_)
	{
		max_iter = max_iter_;
	}
	const int PolicyIteration::get_iter()
	{
		return iter;
	}

	// set state numbers
	void PolicyIteration::set_state_num(const int N_)
	{
		N = N_;
	}

	const int PolicyIteration::get_state_num()
	{
		return N;
	}

	// set action numbers
	void PolicyIteration::set_action_num(const int M_)
	{
		M = M_;
	}

	const int PolicyIteration::get_action_num()
	{
		return M;
	}

	void PolicyIteration::setDiscount(const int discount_)
	{
		discount = discount_;
	}


	//********************* private functions ***********************************//
	void PolicyIteration::computePR()
	{
		PR = reward;
	} 

	void PolicyIteration::bellman_operator()
	{
		Eigen::MatrixXd Q(N, M);
		//std::cout<<"start"<<std::endl;
		//std::cout<<value<<std::endl;
		//std::cout<<discount<<std::endl;
		for (int j = 0; j < M; j++)
		{
			Q.col(j).array() = PR.col(j).array() + discount * (trans_3d[j] * value).array();
		}
		//igl::mat_max(Q, value, policy);
		Eigen::MatrixXd::Index maxIndex;
		for (int i = 0; i< N; i++)
		{
			value(i) = Q.row(i).maxCoeff(&maxIndex);
			policy(i) = maxIndex;
		}
	}

	void PolicyIteration::eval_policy_matrix()
	{
		Eigen::MatrixXd Ppolicy(N, N);
		Eigen::MatrixXd PRpolicy(N, 1);
		//computePpolicyPRpolicy();
		for (int j = 0; j < M; j++)
		{
			for (int i = 0; i < N; i++)
			{
				if (policy(i) == j)
				{
					Ppolicy.row(i).array() = trans_3d[j].row(i).array();
					PRpolicy(i) = PR(i,j);
				}
			}
		}
		//std::cout<<Ppolicy<<std::endl;
		//std::cout<<PRpolicy<<std::endl;
		//value = (Eigen::MatrixXd::Identity(N, N) - discount * Ppolicy).inverse() * PRpolicy;
	    //std::cout<<value<<std::endl;

	     //Vector3f x = A.colPivHouseholderQr().solve(b);
		// http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html
		value = (Eigen::MatrixXd::Identity(N, N) - discount * Ppolicy).fullPivLu().solve(PRpolicy);
	}

	void PolicyIteration::eval_policy_iterative()
	{
		
	}

}


