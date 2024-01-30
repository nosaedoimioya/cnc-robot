function Jacobi = ur5e_jacobian(Q)
%UR5E_JACOBIAN
%    Jacobi = UR5E_JACOBIAN(Q)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    15-Sep-2022 12:50:06

% Positions
q_1 = Q(1); q_2 = Q(2); q_3 = Q(3);
q_4 = Q(4); q_5 = Q(5);

t2 = cos(q_1);
t3 = cos(q_2);
t4 = cos(q_3);
t5 = cos(q_4);
t6 = cos(q_5);
t7 = sin(q_1);
t8 = sin(q_2);
t9 = sin(q_3);
t10 = sin(q_4);
t11 = sin(q_5);
t12 = t2.^2;
t13 = t2.*t3;
t14 = t3.*t4;
t15 = t2.*t8;
t16 = t3.*t7;
t17 = t3.*t9;
t18 = t4.*t8;
t19 = t7.*t8;
t20 = t8.*t9;
t21 = -t2;
t27 = t6.*9.8441e-2;
t28 = t11.*9.8441e-2;
t34 = t2.*t7.*6.123233995736766e-17;
t54 = t6.*6.02777277774323e-18;
t55 = t5.*t11.*6.02777277774323e-18;
t56 = t10.*t11.*6.02777277774323e-18;
t22 = -t20;
t23 = t17+t18;
t29 = -t28;
t30 = t5.*t27;
t31 = t5.*t28;
t32 = t10.*t27;
t33 = t10.*t28;
t35 = t15.*6.123233995736766e-17;
t36 = t19.*6.123233995736766e-17;
t37 = t12.*6.123233995736766e-17;
t40 = t15+t34;
t57 = -t56;
t75 = t54-9.970000000000001e-2;
t24 = t14+t22;
t25 = t10.*t23;
t38 = -t36;
t39 = -t37;
t41 = t16+t35;
t44 = t9.*t40;
t46 = t4.*t40;
t68 = t32+t55;
t69 = t30+t57;
t76 = t5.*t75;
t77 = t10.*t75;
t26 = t5.*t24;
t42 = t19+t39;
t43 = t13+t38;
t45 = t9.*t41;
t47 = t4.*t41;
t70 = t4.*t68;
t71 = t9.*t68;
t72 = t4.*t69;
t73 = t9.*t69;
t78 = -t76;
t79 = t31+t77;
t83 = -t4.*(t76-t10.*t11.*9.8441e-2);
t84 = -t9.*(t76-t10.*t11.*9.8441e-2);
t85 = t9.*(t76-t10.*t11.*9.8441e-2);
t48 = t9.*t43;
t49 = t4.*t42;
t50 = t9.*t42;
t51 = t4.*t43;
t52 = -t47;
t74 = -t71;
t80 = t33+t78;
t81 = t4.*t79;
t82 = t9.*t79;
t86 = t79+3.922e-1;
t89 = t70+t73;
t92 = -t3.*(t71-t72);
t95 = t8.*(t71-t72).*(-6.123233995736766e-17);
t53 = -t51;
t58 = t46+t48;
t59 = t45+t49;
t61 = t50+t52;
t64 = -t10.*(t47-t50);
t66 = t10.*(t47-t50);
t87 = t4.*t86;
t88 = t9.*t86;
t90 = t72+t74;
t91 = t8.*t89;
t94 = t2.*t89.*6.123233995736766e-17;
t96 = t82+t83;
t97 = t81+t85;
t60 = t44+t53;
t62 = t5.*t59;
t63 = t5.*t58;
t93 = -t91;
t98 = t3.*t96;
t99 = t8.*t97;
t100 = t83+t88;
t101 = t85+t87;
t102 = t3.*(t88-t4.*(t76-t10.*t11.*9.8441e-2));
t103 = t8.*(t88-t4.*(t76-t10.*t11.*9.8441e-2));
t109 = t8.*t96.*6.123233995736766e-17;
t111 = t2.*t97.*6.123233995736766e-17;
t112 = t2.*(t88-t4.*(t76-t10.*t11.*9.8441e-2)).*6.123233995736766e-17;
t118 = t29+t94+t95;
t65 = t10.*t60;
t104 = t8.*t101;
t105 = -t103;
t106 = t101+1.7e+1./4.0e+1;
t110 = -t109;
t113 = t103.*6.123233995736766e-17;
t115 = t2.*t101.*6.123233995736766e-17;
t117 = t92+t93;
t119 = t98+t99;
t67 = -t65;
t107 = t3.*t106;
t108 = t8.*t106;
t114 = -t113;
t120 = t102+t104;
t123 = t110+t111;
t116 = t108.*6.123233995736766e-17;
t121 = t102+t108;
t122 = t105+t107;
t124 = t114+t115;
t125 = t27+t112+t116+1.333e-1;
et1 = t7.*3.749399456654644e-33+t63.*6.123233995736766e-17;
et2 = t65.*(-6.123233995736766e-17)-t6.*(t63+t67).*6.123233995736766e-17+t6.*t7+t11.*(t5.*t60+t10.*t58);
et3 = t2.*(-3.749399456654644e-33)+t62.*6.123233995736766e-17;
et4 = t66.*6.123233995736766e-17-t6.*(t62+t66).*6.123233995736766e-17+t6.*t21+t11.*(t10.*t59-t5.*(t47-t50));
et5 = t6.*6.123233995736766e-17+t25.*6.123233995736766e-17-t26.*6.123233995736766e-17;
et6 = t6.*(t25-t26).*(-6.123233995736766e-17)-t11.*(t5.*t23+t10.*t24);
et7 = 2.295845021658468e-49;
mt1 = [t7.^2.*(t88-t4.*(t76-t10.*t11.*9.8441e-2)).*(-6.123233995736766e-17)+t2.*t125-t7.*(t103-t107),t7.*t125-t21.*(t103-t107)+t34.*(t88-t4.*(t76-t10.*t11.*9.8441e-2)),0.0,0.0,0.0];
mt2 = [1.0,t16.*t106.*6.123233995736766e-17-t21.*t121,t13.*t106.*(-6.123233995736766e-17)+t7.*t121,t103-t107,t7,t21,6.123233995736766e-17,-t21.*t120-t7.*(t113-t115)];
mt3 = [t7.*t120-t21.*(t113-t115),t103-t3.*t101,t7,t21,6.123233995736766e-17,-t21.*t119-t7.*(t109-t111),t7.*t119-t21.*(t109-t111),-t3.*t97+t8.*t96,t7,t21,6.123233995736766e-17];
mt4 = [-t21.*(t91+t3.*(t71-t72))-t7.*(t28-t94+t8.*(t71-t72).*6.123233995736766e-17),t7.*(t91+t3.*(t71-t72))-t21.*(t28-t94+t8.*(t71-t72).*6.123233995736766e-17)];
mt5 = [t11.*(-6.02777277774323e-18)-t3.*t89+t8.*(t71-t72),t7.*6.123233995736766e-17+t63+t67];
mt6 = [t2.*(-6.123233995736766e-17)+t62+t66,t25-t26+3.749399456654644e-33,0.0,0.0,0.0,et1+et2,et3+et4,et5+et6+et7];
Jacobi = reshape([mt1,mt2,mt3,mt4,mt5,mt6],6,6);