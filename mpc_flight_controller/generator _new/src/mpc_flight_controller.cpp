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

	Control             psid  ;  // command roll
    Control             thetad;  // command theta
    Control             vzd   ;  // command vertical velocity

    const float     pi = 3.14159265359; // pi
	const double     g = 9.81;  // the gravitational constant 
    
	// identification result
    const double Ax =   0.1840; // air drag
    const double Ay =   0.2007; // air drag
    const double va1 =  -2.292; // vertical velocity model
    const double va2 =  -2.550; // vertical velocity model
    const double va3 =   9.742; // vertical velocity model
    const double va4 =  -5.845; // vertical velocity model
    const double vb1 = 0.00642; // vertical velocity model
    const double vb2 = -0.6570; // vertical velocity model
    const double vc1 =  17.650; // vertical velocity model
    const double vc2 = -0.3225; // vertical velocity model
    const double vd =        0; // vertical velocity model
    const double ra =  -6.7529; // roll model
    const double rb =  6.86585; // roll model
    const double pa =  -6.7317; // pitch model
    const double pb =  6.85417; // pitch model

    // Model equations:
	DifferentialEquation f; 

	f << dot(  x  ) == vx;
	f << dot(  y  ) == vy;
    f << dot(  z  ) == vc1 * z1 + vc2 * z2 + vd * vzd;
	f << dot(  vx ) == - Ax * vx + g * theta;
    f << dot(  vy ) == - Ay * vy + g * psi;
	f << dot(  z1 ) == va1 * z1 + va2 * z2 + vb1 * vzd;
    f << dot(  z2 ) == va3 * z1 + va4 * z2 + vb2 * vzd;
	f << dot( psi ) == ra * psi + rb * psid;
    f << dot(theta) == pa * theta + pb * thetad;

	//
	// Optimal Control Problem
	//
	const float TEnd = 2.5;
	const float Ts = 1 / 15.0;

	OCP ocp( 0.0,  TEnd , TEnd / Ts);

	ocp.minimizeLagrangeTerm( x*x + y*y + z*z + vx*vx + vy*vy);

	ocp.subjectTo( f );

	// ocp.setNOD(1);	// set number of online data

	// ocp.subjectTo( -1 <= vx <= 1 ); 				// m/s
	// ocp.subjectTo( -1 <= vy <= 1 ); 				// m/s
	// ocp.subjectTo( -0.1 <= vx_b ); 					// moving forward
	ocp.subjectTo( -pi / 36 <= psid <= pi / 36 ); 	// deg
	ocp.subjectTo( -pi / 36 <= thetad <= pi / 36 ); // deg
    ocp.subjectTo( -1 <= vzd <= 1 ); 				// deg
	// ocp.subjectTo( -pi / 2 <= phird <= pi / 2 ); 	// 90 deg/s

	// Export the code:
	OCPexport mpc( ocp );
	
	mpc.set( HESSIAN_APPROXIMATION,       EXACT_HESSIAN  		);
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING 	);
	mpc.set( INTEGRATOR_TYPE,             INT_RK4      			);
	mpc.set( QP_SOLVER,                   QP_QPOASES    		);
	mpc.set( HOTSTART_QP,                 NO             		);
	mpc.set( GENERATE_TEST_FILE,          YES            		);
	mpc.set( GENERATE_MAKE_FILE,          YES            		);
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO            		);
	mpc.set( SPARSE_QP_SOLUTION, 		  FULL_CONDENSING_N2	);
	mpc.set( DYNAMIC_SENSITIVITY, 		  SYMMETRIC				);
	mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO 					);
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, YES 				);


    // mpc.set( CG_USE_OPENMP, NO);
    // mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
    // mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);

	if (mpc.exportCode( "mpc_flight_controller_export" ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
