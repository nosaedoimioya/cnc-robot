function R_40 = threeLinkUr5e_40_rotation_matrix(Q)
%threeLinkUr5e_40_rotation_matrix
%    R_40 = threeLinkUr5e_40_rotation_matrix(Q)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    25-Jan-2024 15:59:43
q_1 = Q(1); q_2 = Q(2); q_3 = Q(3);

t2 = cos(q_1);
t3 = cos(q_2);
t4 = cos(q_3);
t5 = sin(q_1);
t6 = sin(q_2);
t7 = sin(q_3);
t8 = t2.*t3;
t9 = t2.*t6;
t10 = t3.*t5;
t11 = t5.*t6;
t12 = -t11;
t13 = t8.*6.123233995736766e-17;
t14 = t9.*6.123233995736766e-17;
t15 = t10.*6.123233995736766e-17;
t16 = t11.*6.123233995736766e-17;
t17 = -t16;
t18 = t9+t15;
t19 = t10+t14;
t21 = t12+t13;
t20 = t8+t17;
R_40 = reshape([t4.*t20-t7.*t18,t4.*t19+t7.*t21,t3.*t7+t4.*t6,-t4.*t18-t7.*t20,t4.*t21-t7.*t19,t3.*t4-t6.*t7,t5,-t2,6.123233995736766e-17],[3,3]);
