function Jacobi_dot = ur5e_jacobian_derivative(DQ,Q)
%UR5E_JACOBIAN_DERIVATIVE
%    Jacobi_dot = UR5E_JACOBIAN_DERIVATIVE(DQ,Q)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    26-Oct-2023 13:23:05

% Position derivatives (velocities)
dq_1 = DQ(1); dq_2 = DQ(2); dq_3 = DQ(3);
dq_4 = DQ(4); dq_5 = DQ(5);

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
t12 = dq_1.*t2;
t13 = dq_1.*t7;
t14 = t2.*t3;
t15 = t3.*t4;
t16 = t2.*t8;
t17 = t3.*t7;
t18 = t3.*t9;
t19 = t4.*t8;
t20 = t7.*t8;
t21 = t8.*t9;
t34 = t6.*9.8441e-2;
t35 = t11.*9.8441e-2;
t41 = t10.*t11.*(-9.8441e-2);
t66 = t6.*6.02777277774323e-18;
t68 = t5.*t11.*6.02777277774323e-18;
t70 = t10.*t11.*6.02777277774323e-18;
t22 = -t20;
t23 = -t21;
t24 = t18+t19;
t36 = -t35;
t37 = t5.*t34;
t38 = t5.*t35;
t39 = t10.*t34;
t40 = t10.*t35;
t42 = t14.*6.123233995736766e-17;
t43 = t16.*6.123233995736766e-17;
t44 = t17.*6.123233995736766e-17;
t45 = t20.*6.123233995736766e-17;
t67 = t5.*t66;
t69 = t10.*t66;
t71 = -t70;
t105 = t66-9.970000000000001e-2;
t25 = t15+t23;
t26 = t5.*t24;
t27 = t10.*t24;
t46 = -t45;
t47 = t16+t44;
t48 = t17+t43;
t50 = t22+t42;
t86 = t38+t69;
t87 = t39+t68;
t88 = t37+t71;
t89 = t41+t67;
t106 = t5.*t105;
t107 = t10.*t105;
t28 = t5.*t25;
t29 = t10.*t25;
t30 = -t27;
t49 = t14+t46;
t51 = t9.*t47;
t52 = t9.*t48;
t53 = t4.*t47;
t54 = t4.*t48;
t55 = t26.*6.123233995736766e-17;
t59 = t9.*t50;
t61 = t4.*t50;
t90 = t4.*t86;
t91 = t4.*t87;
t92 = t9.*t86;
t93 = t9.*t87;
t94 = t4.*t88;
t95 = t4.*t89;
t96 = t9.*t88;
t97 = t9.*t89;
t108 = -t106;
t110 = t38+t107;
t114 = -t4.*(t41+t106);
t115 = -t9.*(t41+t106);
t116 = t9.*(t41+t106);
t31 = t26+t29;
t32 = t28+t30;
t33 = -t11.*(t27-t28);
t56 = t9.*t49;
t57 = t4.*t49;
t58 = -t55;
t60 = t29.*6.123233995736766e-17;
t64 = -t61;
t73 = t54+t59;
t98 = -t92;
t99 = -t93;
t111 = t40+t108;
t112 = t4.*t110;
t113 = t9.*t110;
t117 = t110+3.922e-1;
t127 = t90+t97;
t128 = t91+t96;
t134 = -t3.*(t93-t94);
t135 = -t8.*(t93-t94);
t136 = -t8.*(t92-t95);
t146 = t3.*(t93-t94).*(-6.123233995736766e-17);
t147 = t3.*(t92-t95).*(-6.123233995736766e-17);
t148 = t8.*(t93-t94).*(-6.123233995736766e-17);
t149 = t3.*(t92-t95).*6.123233995736766e-17;
t62 = -t60;
t63 = -t57;
t65 = t6.*t31.*6.123233995736766e-17;
t72 = t53+t56;
t75 = t52+t64;
t78 = t5.*t73;
t79 = t10.*t73;
t119 = t4.*t117;
t120 = t9.*t117;
t129 = t94+t99;
t130 = t95+t98;
t131 = t3.*t127;
t132 = t3.*t128;
t133 = t8.*t128;
t143 = t8.*t127.*6.123233995736766e-17;
t150 = t113+t114;
t151 = t112+t116;
t74 = t51+t63;
t76 = t5.*t72;
t77 = t10.*t72;
t82 = t5.*t75;
t83 = t10.*t75;
t101 = t78.*6.123233995736766e-17;
t109 = t33+t58+t62+t65;
t137 = -t133;
t142 = t132.*6.123233995736766e-17;
t144 = t133.*6.123233995736766e-17;
t152 = t3.*t150;
t153 = t8.*t150;
t154 = t3.*t151;
t155 = t8.*t151;
t157 = t114+t120;
t158 = t116+t119;
t159 = t3.*(t120-t4.*(t41+t106));
t160 = t8.*(t120-t4.*(t41+t106));
t179 = t131+t136;
t180 = t132+t135;
t184 = -dq_5.*(t133+t3.*(t93-t94));
t185 = -t2.*(t133+t3.*(t93-t94));
t186 = -t7.*(t133+t3.*(t93-t94));
t187 = dq_5.*(t133+t3.*(t93-t94));
t192 = t34+t143+t149;
t80 = t5.*t74;
t81 = t10.*t74;
t85 = -t83;
t100 = t77.*6.123233995736766e-17;
t103 = t83.*6.123233995736766e-17;
t122 = t79+t82;
t145 = -t144;
t156 = -t153;
t161 = t3.*t158;
t162 = t8.*t158;
t163 = -t160;
t164 = t158+1.7e+1./4.0e+1;
t167 = t152.*6.123233995736766e-17;
t168 = t153.*6.123233995736766e-17;
t170 = t154.*6.123233995736766e-17;
t171 = t155.*6.123233995736766e-17;
t172 = t159.*6.123233995736766e-17;
t173 = t160.*6.123233995736766e-17;
t181 = t134+t137;
t182 = t2.*t180;
t183 = t7.*t180;
t189 = -t2.*(t144+t3.*(t93-t94).*6.123233995736766e-17);
t190 = -t7.*(t144+t3.*(t93-t94).*6.123233995736766e-17);
t191 = t2.*(t144+t3.*(t93-t94).*6.123233995736766e-17);
t193 = t36+t142+t148;
t194 = -t2.*(t35-t142+t8.*(t93-t94).*6.123233995736766e-17);
t195 = -t7.*(t35-t142+t8.*(t93-t94).*6.123233995736766e-17);
t196 = t7.*(t35-t142+t8.*(t93-t94).*6.123233995736766e-17);
t199 = t152+t155;
t204 = -t2.*(t153-t154);
t205 = -t7.*(t153-t154);
t84 = -t81;
t102 = t80.*6.123233995736766e-17;
t104 = -t103;
t118 = t77+t80;
t123 = t78+t85;
t125 = t11.*t122;
t165 = t3.*t164;
t166 = t8.*t164;
t169 = -t168;
t174 = -t173;
t175 = t161.*6.123233995736766e-17;
t176 = t162.*6.123233995736766e-17;
t188 = t145+t146;
t200 = dq_4.*t199;
t201 = t154+t156;
t202 = t2.*t199;
t203 = t7.*t199;
t206 = t159+t162;
t208 = t161+t163;
t211 = -t2.*(t160-t161);
t212 = -t7.*(t160-t161);
t219 = t167+t171;
t240 = t182+t190;
t242 = t183+t191;
t244 = t186+t194;
t245 = t185+t196;
t121 = t76+t84;
t138 = t6.*t118.*6.123233995736766e-17;
t140 = t6.*t123.*6.123233995736766e-17;
t177 = t165.*6.123233995736766e-17;
t178 = t166.*6.123233995736766e-17;
t207 = dq_3.*t206;
t209 = t2.*t206;
t210 = t7.*t206;
t213 = t159+t166;
t214 = t163+t165;
t217 = -t2.*(t160-t165);
t218 = -t7.*(t160-t165);
t220 = t169+t170;
t221 = t2.*t219;
t222 = t7.*t219;
t227 = t172+t176;
t228 = t174+t175;
t241 = dq_5.*t240;
t243 = dq_5.*t242;
t124 = t11.*t121;
t139 = -t138;
t141 = -t140;
t215 = t2.*t213;
t216 = t7.*t213;
t223 = t2.*t220;
t224 = t7.*t220;
t225 = -t222;
t229 = t2.*t227;
t230 = t7.*t227;
t231 = t2.*t228;
t232 = t7.*t228;
t233 = t172+t178;
t235 = t174+t177;
t248 = t205+t221;
t252 = -dq_4.*(t222+t2.*(t153-t154));
t126 = -t124;
t198 = t101+t104+t125+t141;
t226 = -t223;
t234 = -t230;
t236 = -t231;
t237 = t2.*t235;
t238 = t7.*t235;
t246 = t34+t233+1.333e-1;
t247 = t202+t224;
t250 = dq_4.*t248;
t251 = t204+t225;
t253 = t209+t232;
t254 = t212+t229;
t258 = -dq_3.*(t230+t2.*(t160-t161));
t197 = t100+t102+t126+t139;
t239 = -t237;
t249 = t203+t226;
t255 = t210+t236;
t256 = dq_3.*t254;
t257 = t211+t234;
t259 = t215+t238;
t260 = t216+t239;
et1 = t2.*3.749399456654644e-33-t79.*6.123233995736766e-17;
et2 = t82.*(-6.123233995736766e-17)+t2.*t6+t6.*t122.*6.123233995736766e-17+t11.*t123;
et3 = t7.*3.749399456654644e-33+t76.*6.123233995736766e-17;
et4 = t81.*(-6.123233995736766e-17)+t6.*t7-t6.*t121.*6.123233995736766e-17+t11.*t118;
mt1 = [-dq_4.*t249-dq_3.*t255-dq_2.*t260-dq_5.*(t7.*(t133+t3.*(t93-t94))+t2.*(t35-t142+t8.*(t93-t94).*6.123233995736766e-17))-dq_1.*(t7.*t246+t2.*(t160-t165)),dq_4.*t247+dq_3.*t253+dq_2.*t259+dq_5.*(t195+t2.*(t133+t3.*(t93-t94)))+dq_1.*(t2.*t246-t7.*(t160-t165)),0.0,0.0,0.0,0.0,t241+t252+t258-dq_1.*t260-dq_2.*(t7.*t233+t2.*(t160-t165)),t243+t250+t256+dq_1.*t259+dq_2.*(t218+t2.*t233)];
mt2 = [t187+t200+t207+dq_2.*t213,t12,t13,0.0,t241+t252+t258-dq_2.*(t230+t2.*(t160-t161))-dq_1.*t255,t243+t250+t256+dq_1.*t253+dq_2.*t254,t187+t200+t207+dq_2.*t206,t12,t13,0.0,t241+t252-dq_2.*(t222+t2.*(t153-t154))-dq_3.*(t222+t2.*(t153-t154))-dq_1.*t249,t243+t250+dq_1.*t247+dq_2.*t248+dq_3.*t248,t187+t200+dq_2.*t199+dq_3.*t199,t12,t13,0.0];
mt3 = [dq_2.*t240+dq_3.*t240+dq_4.*t240-dq_1.*(t7.*(t133+t3.*(t93-t94))+t2.*(t35-t142+t8.*(t93-t94).*6.123233995736766e-17))+dq_5.*(t2.*t179-t7.*t192),dq_2.*t242+dq_3.*t242+dq_4.*t242+dq_5.*(t7.*t179+t2.*t192)+dq_1.*(t195+t2.*(t133+t3.*(t93-t94))),dq_2.*(t133+t3.*(t93-t94))+dq_3.*(t133+t3.*(t93-t94))+dq_4.*(t133+t3.*(t93-t94))+dq_5.*(-t66+t8.*t127+t3.*(t92-t95))];
mt4 = [-dq_2.*t118-dq_3.*t118-dq_4.*t118+dq_1.*(t2.*6.123233995736766e-17-t122),dq_2.*t123+dq_3.*t123+dq_4.*t123+dq_1.*(t7.*6.123233995736766e-17+t121),dq_2.*t31+dq_3.*t31+dq_4.*t31,0.0,0.0,0.0];
mt5 = [-dq_2.*t197-dq_3.*t197-dq_4.*t197+dq_1.*(et1+et2)+dq_5.*(t124.*6.123233995736766e-17-t7.*t11+t6.*t118),dq_2.*t198+dq_3.*t198+dq_4.*t198+dq_1.*(et3+et4)+dq_5.*(t125.*6.123233995736766e-17+t2.*t11-t6.*t123)];
mt6 = [-dq_5.*(t11.*6.123233995736766e-17+t6.*t31-t11.*(t27-t28).*6.123233995736766e-17)+dq_2.*(t55+t60-t65+t11.*(t27-t28))+dq_3.*(t55+t60-t65+t11.*(t27-t28))+dq_4.*(t55+t60-t65+t11.*(t27-t28))];
Jacobi_dot = reshape([mt1,mt2,mt3,mt4,mt5,mt6],6,6);
