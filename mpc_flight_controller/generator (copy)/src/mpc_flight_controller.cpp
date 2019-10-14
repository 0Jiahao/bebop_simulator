#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace std;
#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main( )
{
    USING_NAMESPACE_ACADO

	// prediction horizon and sampling time
	const double Ts = 1 / 10.0;
	const float TEnd = Ts * 25;

    // Variables:
	DifferentialState   x     ;  // position x
	DifferentialState   y     ;  // position y
	DifferentialState   z     ;  // position z
	DifferentialState   vx    ;  // velocity x 
    DifferentialState   vy    ;  // velocity y
    DifferentialState   z1    ;  // velocity z state z1
    DifferentialState   z2    ;  // velocity z state sz2
    DifferentialState   psi   ;  // angle roll
    DifferentialState   theta ;  // angle pitch
	DifferentialState   phi   ;  // angle yaw
	DifferentialState   y1    ;  // yaw state y1
	DifferentialState   y2    ;  // yaw state y2
	DifferentialState   dummy1;  // dummy for obs1
	DifferentialState   dummy2;  // dummy for obs2
	
	// Control command
	Control             psid  ;  // command roll
    Control             thetad;  // command theta
    Control             vzd   ;  // command vertical velocity
	Control   			phird ;  // command yawrate
	// Slack variables
	Control 			sv_obs1;  // slack variable for obstacle 1 
	Control 			sv_obs2;  // slack variable for obstacle 2

	OnlineData 			obs1_x;	 // obstacle's x
	OnlineData 			obs1_y;	 // obstacle's y
	OnlineData 			obs1_h;	 // obstacle's heading
	OnlineData 			obs1_a;	 // obstacle's major axis
	OnlineData 			obs1_b;	 // obstacle's minor axis

	OnlineData 			obs2_x;	 // obstacle's x
	OnlineData 			obs2_y;	 // obstacle's y
	OnlineData 			obs2_h;	 // obstacle's heading
	OnlineData 			obs2_a;	 // obstacle's major axis
	OnlineData 			obs2_b;	 // obstacle's minor axis
	// OnlineData 			obs2_x;	 // obstacle's x
	// OnlineData 			obs2_y;	 // obstacle's y
	// OnlineData  		obs_idx; // index for obstacle avoidance
	
    const float     pi = 3.14159265359; // pi
	const float     g = 9.80665;        // gravity

    // Model equations:
	DiscretizedDifferentialEquation f(Ts); 

	f << next(  x  ) == x + Ts * vx;
	f << next(  y  ) == y + Ts * vy;
    f << next(  z  ) == z + 1.067248658575422 * z1 - 0.100493327248945 * z2 + 0.001919730358230 * vzd;
	f << next(  vx ) == 0.987808262200150 * vx + tan(theta) / cos(psi) * g * Ts;
    f << next(  vy ) == 0.986709114307428 * vy - tan(psi) * g * Ts;
	f << next(  z1 ) == 0.814844314440081 * z1 - 0.127542464394678 * z2 + 0.003479876523758 * vzd;
    f << next(  z2 ) == 0.487262230640373 * z1 + 0.637135147383496 * z2 - 0.035522579058221 * vzd;
	f << next( psi ) == 0.606574521972616 * psi + 0.361974893073679 * psid;
    f << next(theta) == 0.591025012040307 * theta + 0.377509106693901 * thetad;
	f << next( phi ) == phi + 0.580946305515318 * y1 - 0.002380060951283 * y2;
	f << next(  y1 ) == 0.936465349398944 * y1 - 0.046804990378125 * y2 + 5.125900721967114e-04 * phird;
	f << next(  y2 ) == 0.236914366025587 * y1 + 0.820329015243120 * y2 - 0.013647220727940 * phird;
	f << next(  dummy1 ) == sv_obs1;
	f << next(  dummy2 ) == sv_obs2;
	

    // Reference functions and weighting matrices:
	Function h, hN;
	h << psid << thetad << vzd << phird << sv_obs1 << sv_obs2;
	hN << x << y << z << vx << vy << phi;

	DMatrix W = eye<double>(h.getDim());
	W(4,4) = 10000;
	W(5,5) = 10000;
	DMatrix WN = eye<double>(hN.getDim());
	WN(2,2) = 100;
	WN(3,3) = 10;
	WN(4,4) = 10;
	WN(5,5) = 100;
	//
	// Optimal Control Problem
	//

	OCP ocp( 0.0,  TEnd , TEnd / Ts);

	Expression d2_obs1 = (x - obs1_x) * (x - obs1_x) + (y - obs1_y) * (y - obs1_y);
	// Expression d2_obs2 = (x - obs2_x) * (x - obs2_x) + (y - obs2_y) * (y - obs2_y);

	Expression R_obs1(2,2); // heading of the obstacle
	R_obs1(0,0) = cos(obs1_h);
	R_obs1(0,1) = sin(obs1_h);
	R_obs1(1,0) =-sin(obs1_h);
	R_obs1(1,1) = cos(obs1_h);

	Expression v_q2obs1(2,1); // vector from quadrotor to obstacle
	v_q2obs1(0) = x - obs1_x;
	v_q2obs1(1) = y - obs1_y;

	Expression S_obs1(2,2); // shape of the obstacle
	S_obs1(0,0) = 1 / ((obs1_a + 0.5) * (obs1_a + 0.5));
	S_obs1(0,1) = 0;
	S_obs1(1,0) = 0;
	S_obs1(1,1) = 1 / ((obs1_b + 0.5) * (obs1_b + 0.5));

	Expression pseudoDist1; // in < 1; out > 1; on = 1
	pseudoDist1 = v_q2obs1.transpose() * R_obs1.transpose() * S_obs1 * R_obs1 * v_q2obs1;

	Expression R_obs2(2,2); // heading of the obstacle
	R_obs2(0,0) = cos(obs2_h);
	R_obs2(0,1) = sin(obs2_h);
	R_obs2(1,0) =-sin(obs2_h);
	R_obs2(1,1) = cos(obs2_h);

	Expression v_q2obs2(2,1); // vector from quadrotor to obstacle
	v_q2obs2(0) = x - obs2_x;
	v_q2obs2(1) = y - obs2_y;

	Expression S_obs2(2,2); // shape of the obstacle
	S_obs2(0,0) = 1 / ((obs2_a + 0.3) * (obs2_a + 0.3));
	S_obs2(0,1) = 0;
	S_obs2(1,0) = 0;
	S_obs2(1,1) = 1 / ((obs2_b + 0.3) * (obs2_b + 0.3));

	Expression pseudoDist2; // in < 1; out > 1; on = 1
	pseudoDist2 = v_q2obs2.transpose() * R_obs2.transpose() * S_obs2 * R_obs2 * v_q2obs2;

	ocp.setNOD(10);

	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);

	ocp.subjectTo( f );

	ocp.subjectTo( -pi / 36 <= psid <= pi / 36 ); 	// deg
	ocp.subjectTo( -pi / 36 <= thetad <= pi / 36 ); // deg
    ocp.subjectTo( -1 <= vzd <= 1 ); 				// deg
	ocp.subjectTo( -pi / 2 <= phird <= pi / 2 ); 	// 90 deg/s
	
	ocp.subjectTo( pseudoDist1 + sv_obs1 >= 1);
	ocp.subjectTo( 0 <= sv_obs1 );

	ocp.subjectTo( pseudoDist2 + sv_obs2 >= 1);
	ocp.subjectTo( 0 <= sv_obs2 );

	// ocp.subjectTo( 1 <= d2_obs2 + sv_obs2 );
	// ocp.subjectTo( 0.25 <= d2_obs2 );
	// ocp.subjectTo( 0 <= sv_obs2 );

	// Export the code:
	OCPexport mpc( ocp );
	
	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); 
    mpc.set( INTEGRATOR_TYPE, INT_DT);
    mpc.set( QP_SOLVER, QP_QPOASES);
    mpc.set( HOTSTART_QP, YES);
    mpc.set( LEVENBERG_MARQUARDT, 1e-10);
    mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    // mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
	mpc.set( GENERATE_TEST_FILE, BT_TRUE);
    // mpc.set( CG_USE_OPENMP, NO);
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
    mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);

	if (mpc.exportCode( "mpc_flight_controller_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
