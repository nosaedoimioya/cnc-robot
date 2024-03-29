function Jacobi_dot = threeLinkUr5e_jacobian_derivative(DQ,Q)
%threeLinkUr5e_jacobian_derivative
%    Jacobi_dot = threeLinkUr5e_jacobian_derivative(DQ,Q)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    25-Jan-2024 15:59:46
q_1 = Q(1); q_2 = Q(2); q_3 = Q(3);
dq_1 = DQ(1); dq_2 = DQ(2); dq_3 = DQ(3);

t2 = cos(q_1);
t3 = cos(q_2);
t4 = cos(q_3);
t5 = sin(q_1);
t6 = sin(q_2);
t7 = sin(q_3);
t8 = dq_1.*t2;
t9 = dq_1.*t5;
t10 = t4.*3.922e-1;
t11 = t7.*3.922e-1;
t12 = t4.*1.634e-2;
t14 = t7.*1.634e-2;
t13 = -t11;
t15 = t10+t14;
t19 = -t3.*(t11-t12);
t20 = -t6.*(t11-t12);
t45 = t3.*(t11-t12).*(-6.123233995736766e-17);
t47 = t6.*(t11-t12).*(-6.123233995736766e-17);
t48 = t3.*(t11-t12).*6.123233995736766e-17;
t16 = t12+t13;
t17 = t3.*t15;
t18 = t6.*t15;
t22 = t15+1.7e+1./4.0e+1;
t21 = -t18;
t23 = t3.*t22;
t24 = t6.*t22;
t26 = t17+t20;
t30 = -dq_3.*(t18+t3.*(t11-t12));
t32 = -t2.*(t18+t3.*(t11-t12));
t33 = -t5.*(t18+t3.*(t11-t12));
t35 = dq_3.*(t18+t3.*(t11-t12));
t37 = t2.*(t18+t3.*(t11-t12));
t43 = t17.*6.123233995736766e-17;
t44 = t18.*6.123233995736766e-17;
t25 = -t24;
t27 = t19+t21;
t28 = t2.*t26;
t29 = t5.*t26;
t31 = t20+t23;
t38 = t2.*(t23-t6.*(t11-t12));
t39 = t5.*(t23-t6.*(t11-t12));
t40 = -t2.*(t24+t3.*(t11-t12));
t41 = -t5.*(t24+t3.*(t11-t12));
t42 = t2.*(t24+t3.*(t11-t12));
t46 = -t44;
t49 = t23.*6.123233995736766e-17;
t50 = t24.*6.123233995736766e-17;
t52 = t43+t47;
t56 = -t2.*(t44+t48);
t57 = -t5.*(t44+t48);
t34 = -t29;
t36 = t19+t25;
t51 = -t50;
t53 = t45+t46;
t54 = t2.*t52;
t55 = t5.*t52;
t58 = t47+t49;
t62 = t48+t50+1.315e-1;
t63 = t28+t57;
t68 = -dq_3.*(t29+t2.*(t44+t48));
t69 = dq_3.*(t29+t2.*(t44+t48));
t59 = t45+t51;
t60 = t2.*t58;
t61 = t5.*t58;
t64 = t33+t54;
t65 = dq_3.*t63;
t66 = t34+t56;
t67 = t37+t55;
t70 = t41+t60;
t71 = t42+t61;
Jacobi_dot = reshape([dq_3.*(t54-t5.*(t18+t3.*(t11-t12)))+dq_2.*(t60-t5.*(t24+t3.*(t11-t12)))+dq_1.*(t38-t5.*t62),dq_3.*t67+dq_2.*t71+dq_1.*(t39+t2.*t62),0.0,0.0,0.0,0.0,t65+dq_2.*(t38-t5.*(t48+t50))+dq_1.*(t60-t5.*(t24+t3.*(t11-t12))),t69+dq_1.*t71+dq_2.*(t39+t2.*(t48+t50)),t35+dq_2.*(t24+t3.*(t11-t12)),t8,t9,0.0,t65+dq_2.*t63+dq_1.*(t54-t5.*(t18+t3.*(t11-t12))),t69+dq_1.*t67+dq_2.*(t29+t2.*(t44+t48)),t35+dq_2.*(t18+t3.*(t11-t12)),t8,t9,0.0],[6,3]);
