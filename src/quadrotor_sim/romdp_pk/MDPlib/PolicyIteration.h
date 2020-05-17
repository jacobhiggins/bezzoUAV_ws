#ifndef _MDP_POLICYITERATION_HEADER__
#define _MDP_POLICYITERATION_HEADER__


#include <eigen3/Eigen/Dense>
#include "header/newType.h"
#include "model.h"

namespace MDP
{
	class PolicyIteration
	{
		public:
			PolicyIteration(Model &model);
			~PolicyIteration();
			Policy_and_Value eval();

			void set_eval_type(const int eval_type_);
			const int get_eval_type();
			void set_max_iter(const int max_iter_);
			const int get_iter();
			void set_state_num(const int N_);
			const int get_state_num();
			void set_action_num(const int M_);
			const int get_action_num();
			void setDiscount(const int discount_);

			void set_transition_matrix(Eigen::MatrixXd *trans_3d_);
			void set_reward_matrix(const Eigen::MatrixXd reward_);
			void update_transition_matrix(const int c1_, const int c2_, const int c3_);
			void update_reward_matrix(const int c1_, const int c2_, const int c3);

			void reset();

		private:

			// variables
			// scalars
			double discount;
			int M, N;
			int eval_type, max_iter;
			int iter;

			// vectors
			//ValueVec Value;
			//PolicyVec Policy;
			Eigen::MatrixXd value;
			Eigen::MatrixXd policy;


			Eigen::MatrixXd *trans_3d;  // transition matrix   dim = N * N * M
			//Eigen::MatrixXd *r_3d;  // reward matrix       dim = N * N * M
			Eigen::MatrixXd reward;
			Eigen::MatrixXd PR;

			// struct
			//Policy_and_Value pv; // short for policy and value

			// fucntions
			void computePR();
			void bellman_operator();
			void eval_policy_matrix();
			void eval_policy_iterative();
	};
}


#endif
