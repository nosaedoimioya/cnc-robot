function B = threeLinkUr5e_christoffel_off_diagonals(Q)
%threeLinkUr5e_christoffel_off_diagonals
%    B = threeLinkUr5e_christoffel_off_diagonals(Q)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    25-Jan-2024 16:01:05
q_1 = Q(1); q_2 = Q(2); q_3 = Q(3);

t2 = cos(q_1);
t3 = cos(q_2);
t4 = cos(q_3);
t5 = sin(q_1);
t6 = sin(q_2);
t7 = sin(q_3);
t8 = t2.*t3;
t9 = t3.*t4;
t10 = t2.*t6;
t11 = t3.*t5;
t12 = t3.*t7;
t13 = t4.*t6;
t14 = t5.*t6;
t15 = t6.*t7;
t26 = t4.*(1.19e+2./1.0e+3);
t30 = t4.*3.922e-1;
t31 = t7.*3.922e-1;
t40 = t4.*1.634e-2;
t42 = t7.*1.634e-2;
t127 = t6.*1.301187224094063e-17;
t16 = -t14;
t17 = -t15;
t18 = t8.*(1.7e+1./8.0e+1);
t19 = t9./6.25e+2;
t20 = t10.*(1.7e+1./8.0e+1);
t21 = t11.*(1.7e+1./8.0e+1);
t22 = t12./6.25e+2;
t23 = t13./6.25e+2;
t24 = t14.*(1.7e+1./8.0e+1);
t25 = t15./6.25e+2;
t27 = t12+t13;
t32 = t9.*(1.19e+2./1.0e+3);
t33 = t9.*4.62e-2;
t34 = t12.*(1.19e+2./1.0e+3);
t35 = t13.*(1.19e+2./1.0e+3);
t36 = t12.*4.62e-2;
t37 = t13.*4.62e-2;
t38 = t15.*(1.19e+2./1.0e+3);
t39 = t15.*4.62e-2;
t41 = -t31;
t46 = t8.*1.712325;
t47 = t10.*1.712325;
t48 = t11.*1.712325;
t49 = t14.*1.712325;
t50 = t26+1.7e+1./4.0e+1;
t51 = t9.*3.38674e-1;
t52 = t12.*3.38674e-1;
t53 = t13.*3.38674e-1;
t55 = t15.*3.38674e-1;
t62 = t30+t42;
t72 = -t3.*(t31-t40);
t73 = -t6.*(t31-t40);
t93 = t3.*(t31-t40).*(-6.07e+2./2.0e+2);
t95 = t6.*(t31-t40).*(-6.07e+2./2.0e+2);
t118 = t8.*6.123233995736766e-17;
t119 = t10.*6.123233995736766e-17;
t120 = t11.*6.123233995736766e-17;
t121 = t14.*6.123233995736766e-17;
t123 = t9.*9.797174393178826e-20;
t124 = t12.*9.797174393178826e-20;
t125 = t13.*9.797174393178826e-20;
t126 = t15.*9.797174393178826e-20;
t129 = t8.*1.301187224094063e-17;
t130 = t11.*1.301187224094063e-17;
t132 = t9.*7.286648454926752e-18;
t133 = t12.*7.286648454926752e-18;
t134 = t13.*7.286648454926752e-18;
t135 = t15.*7.286648454926752e-18;
t136 = t9.*2.828934106030386e-18;
t137 = t12.*2.828934106030386e-18;
t138 = t13.*2.828934106030386e-18;
t139 = t15.*2.828934106030386e-18;
t151 = t8.*1.048496665174996e-16;
t152 = t11.*1.048496665174996e-16;
t166 = t127+1.1336e-1;
t203 = t3.*(t31-t40).*(-6.123233995736766e-17);
t206 = t6.*(t31-t40).*(-6.123233995736766e-17);
t207 = t3.*(t31-t40).*6.123233995736766e-17;
t28 = -t24;
t29 = -t25;
t43 = t9+t17;
t44 = -t38;
t45 = -t39;
t54 = -t49;
t56 = -t55;
t57 = t3.*t50;
t58 = t6.*t50;
t60 = t22+t23;
t63 = t34+t35;
t64 = t36+t37;
t65 = t40+t41;
t69 = t3.*t62;
t70 = t6.*t62;
t79 = t62+1.7e+1./4.0e+1;
t91 = t52+t53;
t122 = -t121;
t128 = -t126;
t140 = t10+t120;
t141 = t11+t119;
t142 = -t135;
t143 = -t139;
t147 = t16+t118;
t176 = t2.*t166;
t177 = t5.*t166;
t180 = t20+t130;
t209 = t47+t152;
t224 = t133+t134;
t59 = -t57;
t61 = t19+t29;
t66 = t32+t44;
t67 = t33+t45;
t68 = t57.*2.846;
t74 = t2.*t63;
t75 = t5.*t63;
t76 = -t70;
t82 = t3.*t79;
t83 = t6.*t79;
t84 = t34+t58;
t87 = t69.*(6.07e+2./2.0e+2);
t88 = t70.*(6.07e+2./2.0e+2);
t96 = t43.*t60.*2.0;
t97 = t51+t56;
t110 = t43.*t64.*2.0;
t131 = t69+t73;
t145 = t8+t122;
t149 = t7.*t140;
t150 = t7.*t141;
t153 = t4.*t140;
t154 = t4.*t141;
t157 = -t2.*(t70+t3.*(t31-t40));
t158 = -t5.*(t70+t3.*(t31-t40));
t161 = t7.*t147;
t162 = t4.*t147;
t164 = t2.*(t70+t3.*(t31-t40));
t170 = t58.*6.123233995736766e-17;
t175 = t57.*6.123233995736766e-17;
t184 = t28+t129;
t185 = -t177;
t187 = t5.*(t70+t3.*(t31-t40)).*(-6.07e+2./2.0e+2);
t198 = t176.*8.058;
t199 = t69.*6.123233995736766e-17;
t200 = t177.*8.058;
t201 = t70.*6.123233995736766e-17;
t210 = t21+t176;
t211 = t54+t151;
t218 = t43.*t60.*6.123233995736766e-17;
t221 = t43.*t64.*6.123233995736766e-17;
t225 = t132+t142;
t226 = t2.*t224;
t227 = t5.*t224;
t71 = -t68;
t77 = t2.*t66;
t78 = t5.*t66;
t80 = -t75;
t85 = -t83;
t86 = t38+t59;
t89 = t27.*t61.*2.0;
t90 = t2.*t84;
t92 = t5.*t84;
t94 = -t88;
t98 = t74.*2.846;
t99 = t75.*2.846;
t106 = t27.*t67.*2.0;
t108 = t82.*(6.07e+2./2.0e+2);
t111 = -t110;
t144 = t72+t76;
t146 = t2.*t131;
t148 = t5.*t131;
t155 = t7.*t145;
t156 = t73+t82;
t160 = t4.*t145;
t167 = t2.*(t82-t6.*(t31-t40));
t168 = t5.*(t82-t6.*(t31-t40));
t169 = -t162;
t171 = -t2.*(t83+t3.*(t31-t40));
t173 = -t5.*(t83+t3.*(t31-t40));
t181 = -t175;
t182 = t2.*(t83+t3.*(t31-t40));
t186 = t164.*(-6.07e+2./2.0e+2);
t189 = t164.*(6.07e+2./2.0e+2);
t190 = t87+t95;
t197 = t5.*(t83+t3.*(t31-t40)).*(-6.07e+2./2.0e+2);
t204 = -t200;
t205 = -t201;
t212 = t82.*6.123233995736766e-17;
t213 = t83.*6.123233995736766e-17;
t215 = t18+t185;
t216 = t27.*t61.*6.123233995736766e-17;
t219 = -t218;
t220 = t27.*t67.*6.123233995736766e-17;
t222 = t48+t198;
t228 = t2.*t225;
t229 = t5.*t225;
t231 = t133+t170;
t232 = t154+t161;
t234 = t226.*2.846;
t235 = t227.*2.846;
t242 = -t2.*(t142+t175);
t243 = -t5.*(t142+t175);
t244 = t5.*(t142+t175);
t260 = -(t82-t6.*(t31-t40)).*(t88+t3.*(t31-t40).*(6.07e+2./2.0e+2));
t261 = t2.*(t142+t175).*(-2.846);
t282 = t199+t206;
t290 = -t2.*(t201+t207);
t291 = -t5.*(t201+t207);
t300 = t2.*(t201+t207).*(-6.07e+2./2.0e+2);
t301 = t5.*(t201+t207).*(-6.07e+2./2.0e+2);
t81 = -t77;
t100 = t2.*t86;
t101 = t5.*t86;
t102 = t77.*2.846;
t103 = t78.*2.846;
t104 = -t99;
t109 = -t106;
t112 = t90.*2.846;
t113 = t92.*2.846;
t116 = t55+t71;
t159 = -t148;
t163 = t72+t85;
t165 = -t160;
t172 = -t167;
t174 = -t168;
t178 = t146.*(6.07e+2./2.0e+2);
t179 = t148.*(6.07e+2./2.0e+2);
t188 = t86.*t91;
t191 = t167.*(6.07e+2./2.0e+2);
t192 = t168.*(6.07e+2./2.0e+2);
t193 = t93+t94;
t196 = t182.*(-6.07e+2./2.0e+2);
t202 = t182.*(6.07e+2./2.0e+2);
t208 = t95+t108;
t214 = -t213;
t217 = -t216;
t223 = t46+t204;
t230 = t153+t155;
t233 = t135+t181;
t237 = t228.*2.846;
t238 = t229.*2.846;
t239 = t150+t169;
t240 = t2.*t231;
t241 = t5.*t231;
t245 = t231+2.65e-2;
t247 = (t2.*t232)./6.25e+2;
t257 = t2.*t232.*4.62e-2;
t262 = t244.*(-2.846);
t265 = t244.*2.846;
t271 = t61.*t232;
t272 = t78+t226;
t273 = t74+t229;
t277 = t80+t228;
t280 = t67.*t232;
t285 = t203+t205;
t287 = t2.*t282;
t288 = t5.*t282;
t292 = t206+t212;
t296 = -t2.*(t207+t213);
t297 = -t5.*(t207+t213);
t304 = t92+t242;
t308 = t207+t213+1.315e-1;
t309 = t90+t244;
t313 = t2.*(t207+t213).*(-6.07e+2./2.0e+2);
t315 = t5.*(t207+t213).*(-6.07e+2./2.0e+2);
t327 = t146+t291;
t105 = -t102;
t107 = -t101;
t114 = t100.*2.846;
t115 = t101.*2.846;
t183 = -t179;
t194 = -t191;
t195 = -t192;
t236 = t149+t165;
t246 = (t5.*t230)./6.25e+2;
t249 = t2.*t245;
t250 = t5.*t245;
t251 = -t247;
t253 = t5.*t230.*4.62e-2;
t254 = t240.*2.846;
t255 = t241.*2.846;
t256 = (t2.*t239)./6.25e+2;
t263 = -t257;
t266 = t2.*t239.*4.62e-2;
t270 = t60.*t230;
t275 = t60.*t239;
t276 = t64.*t230;
t278 = t81+t227;
t284 = t64.*t239;
t286 = -t280;
t293 = t203+t214;
t294 = t2.*t292;
t295 = t5.*t292;
t298 = t287.*(6.07e+2./2.0e+2);
t299 = t288.*(6.07e+2./2.0e+2);
t302 = t103+t234;
t303 = t98+t238;
t305 = t104+t237;
t307 = t100+t241;
t314 = t2.*t308;
t316 = t5.*t308;
t321 = t113+t261;
t322 = t112+t265;
t328 = t158+t287;
t329 = t159+t290;
t330 = t164+t288;
t331 = t167+t297;
t333 = t174+t296;
t335 = t178+t301;
t341 = t191+t315;
t348 = -t309.*(t102-t235);
t349 = t309.*(t102-t235);
t117 = -t115;
t248 = -t246;
t252 = (t5.*t236)./6.25e+2;
t259 = -t253;
t264 = t5.*t236.*4.62e-2;
t268 = t249.*2.846;
t269 = t250.*2.846;
t274 = t61.*t236;
t279 = -t275;
t281 = -t276;
t283 = t67.*t236;
t306 = t105+t235;
t310 = t294.*(6.07e+2./2.0e+2);
t311 = t295.*(6.07e+2./2.0e+2);
t312 = t107+t240;
t317 = t100+t250;
t318 = t107+t249;
t319 = t314.*(6.07e+2./2.0e+2);
t320 = t316.*(6.07e+2./2.0e+2);
t323 = t114+t255;
t332 = t173+t294;
t334 = t182+t295;
t336 = t183+t300;
t337 = t187+t298;
t338 = t189+t299;
t339 = t168+t314;
t340 = t172+t316;
t342 = t195+t313;
t347 = t302.*t304;
t353 = (t102-t235).*(t101-t249);
t372 = -(t179+t2.*(t201+t207).*(6.07e+2./2.0e+2)).*(t294-t5.*(t83+t3.*(t31-t40)));
t378 = (t167-t316).*(t179+t2.*(t201+t207).*(6.07e+2./2.0e+2));
t258 = -t252;
t267 = -t264;
t289 = -t283;
t324 = t117+t254;
t325 = t114+t269;
t326 = t117+t268;
t343 = t197+t310;
t344 = t202+t311;
t345 = t192+t319;
t346 = t194+t320;
t350 = t302.*t317;
t354 = (t77-t227).*(t115-t268);
t355 = -t353;
t356 = t123+t128+t248+t256;
t359 = t136+t143+t259+t266;
t362 = t27.*(t136-t139+t259+t266);
t373 = t334.*t335;
t375 = t335.*t339;
t377 = (t191-t320).*(t148+t2.*(t201+t207));
t379 = -t378;
t388 = t232.*(t136-t139+t259+t266);
t389 = t236.*(t136-t139+t259+t266);
t394 = t271+t279+t284+t286;
t351 = t272.*t325;
t357 = t124+t125+t251+t258;
t358 = t27.*t356;
t360 = t137+t138+t263+t267;
t364 = -t362;
t368 = t362.*1.224646799147353e-16;
t374 = t327.*t345;
t381 = t232.*t356;
t382 = t236.*t356;
t392 = t270+t274+t281+t289;
t395 = t2.*t394;
t352 = -t351;
t361 = t43.*t357;
t363 = t43.*t360;
t366 = t358.*1.224646799147353e-16;
t369 = -t368;
t376 = -t374;
t380 = t230.*t357;
t384 = t239.*t357;
t385 = -t381;
t386 = -t382;
t387 = t230.*t360;
t390 = t239.*t360;
t393 = t5.*t392;
t365 = -t363;
t367 = t361.*1.224646799147353e-16;
t370 = t363.*1.224646799147353e-16;
t383 = -t380;
t391 = -t390;
t398 = t5.*(t380+t382-t387-t389).*-2.0;
t399 = t2.*(t381-t384-t388+t390).*-2.0;
t371 = -t370;
t396 = t383+t386+t387+t389;
t397 = t384+t385+t388+t391;
et1 = t351.*4.0-t354.*4.0+t374.*4.0-t377.*4.0-t393.*4.0-t395.*4.0+t27.*t61.*2.449293598294706e-16-t27.*t67.*2.449293598294706e-16+t43.*t60.*2.449293598294706e-16;
et2 = t43.*t64.*(-2.449293598294706e-16)-t273.*t321.*2.0+t303.*t304.*2.0+t322.*(t75-t228).*2.0-t309.*(t99-t237).*2.0-t344.*(t287-t5.*(t70+t3.*(t31-t40))).*2.0-t338.*(t294-t5.*(t83+t3.*(t31-t40))).*2.0+t334.*(t298-t5.*(t70+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*2.0+t330.*(t310-t5.*(t83+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*2.0;
et3 = t217+t219+t220+t221+t358+t361+t364+t365+t393+t395-t210.*(t14.*1.048496665174996e-16-t46)+t222.*(t14.*1.301187224094063e-17-t18);
et4 = t3.*t6.*(-6.980486755139913e-19)-t180.*t211.*2.0+t184.*t209.*2.0-t304.*t322.*2.0+t309.*t321.*2.0-t331.*t345+t339.*t341+t3.*(t6.*4.776122516674678e-19-t2.*t141.*7.8e-3+t5.*t145.*7.8e-3);
et5 = -t6.*(t3.*(-4.776122516674678e-19)+t5.*t140.*7.8e-3+t2.*t147.*7.8e-3)+t325.*(t101-t240)+t323.*(t101-t249)-t317.*(t115-t254)-t307.*(t115-t268)+t2.*(t3.*t141.*5.7e-3+t6.*t147.*5.7e-3)+t5.*(t6.*t140.*5.7e-3-t3.*t145.*5.7e-3);
et6 = -t3.*(t6.*1.285879139104721e-19-t2.*t141.*2.1e-3+t5.*t145.*2.1e-3)+t6.*(t3.*(-1.285879139104721e-19)+t5.*t140.*2.1e-3+t2.*t147.*2.1e-3)+t344.*(t294-t5.*(t83+t3.*(t31-t40))).*2.0;
et7 = t334.*(t310-t5.*(t83+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*-2.0-t215.*(t10.*1.048496665174996e-16+t48)+t223.*(t10.*1.301187224094063e-17+t21)+(t191-t320).*(t168+t2.*(t207+t213))-(t167-t316).*(t192+t2.*(t207+t213).*(6.07e+2./2.0e+2));
et8 = t188+t260+t347+t349+t366+t367+t369+t371+t372+t373+t398+t399+t190.*(t83+t3.*(t31-t40))-t208.*(t70+t3.*(t31-t40)).*3.0-(t148+t2.*(t201+t207)).*(t310-t5.*(t83+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*3.0+(t168+t2.*(t207+t213)).*(t298-t5.*(t70+t3.*(t31-t40)).*(6.07e+2./2.0e+2))-(t192+t2.*(t207+t213).*(6.07e+2./2.0e+2)).*(t287-t5.*(t70+t3.*(t31-t40)))+t63.*t116.*3.0+t84.*t97+t272.*t321.*3.0-t273.*t323+t303.*t307-t331.*t338+t327.*t344.*3.0+t330.*t341+t322.*(t77-t227).*3.0-t66.*(t52+t58.*2.846);
et9 = -t131.*(t83.*(6.07e+2./2.0e+2)+t3.*(t31-t40).*(6.07e+2./2.0e+2))-(t75-t228).*(t115-t254)+(t99-t237).*(t101-t240);
et10 = t188+t260+t347+t349+t366+t367+t369+t371+t372+t373+t398+t399-t190.*(t70+t3.*(t31-t40)).*3.0+t208.*(t70+t3.*(t31-t40))-t131.*(t88+t3.*(t31-t40).*(6.07e+2./2.0e+2))-(t148+t2.*(t201+t207)).*(t298-t5.*(t70+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*3.0-(t179+t2.*(t201+t207).*(6.07e+2./2.0e+2)).*(t287-t5.*(t70+t3.*(t31-t40)))+(t148+t2.*(t201+t207)).*(t310-t5.*(t83+t3.*(t31-t40)).*(6.07e+2./2.0e+2))-t66.*t91-t63.*t97.*3.0-t63.*t116-t272.*t321+t327.*t338.*3.0+t330.*t335-t327.*t344+t302.*(t75-t228);
et11 = t303.*(t77-t227).*3.0+t272.*(t99-t237).*3.0+t273.*(t102-t235)-t322.*(t77-t227);
mt1 = [t89+t96+t109+t111+t3.*t6.*2.28e-2-t180.*t223.*2.0+t184.*t222.*2.0+t210.*t211.*2.0-t209.*t215.*2.0+t309.*t325.*2.0+t317.*t322.*2.0+t321.*(t101-t249).*2.0+t304.*(t115-t268).*2.0-t344.*(t167-t316).*2.0-t334.*(t191-t320).*2.0+t345.*(t294-t5.*(t83+t3.*(t31-t40))).*2.0+t339.*(t310-t5.*(t83+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*2.0,et3+et4+et5+et6+et7];
mt2 = [t217+t219+t220+t221+t350+t352+t354+t355+t358+t361+t364+t365+t375+t376+t377+t379+t393+t395-t273.*t321-t303.*t304.*3.0+t322.*(t75-t228)+t309.*(t99-t237).*3.0-t344.*(t287-t5.*(t70+t3.*(t31-t40)))+t338.*(t294-t5.*(t83+t3.*(t31-t40))).*3.0-t334.*(t298-t5.*(t70+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*3.0+t330.*(t310-t5.*(t83+t3.*(t31-t40)).*(6.07e+2./2.0e+2))];
mt3 = [t89+t96+t109+t111+t273.*t325.*2.0+t303.*t317.*2.0-t338.*(t167-t316).*2.0-t330.*(t191-t320).*2.0+(t75-t228).*(t115-t268).*2.0+(t99-t237).*(t101-t249).*2.0+t345.*(t287-t5.*(t70+t3.*(t31-t40))).*2.0+t339.*(t298-t5.*(t70+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*2.0];
mt4 = [t217+t219+t220+t221+t350+t352+t354+t355+t358+t361+t364+t365+t375+t376+t377+t379+t393+t395+t273.*t321.*3.0+t303.*t304-t322.*(t75-t228).*3.0-t309.*(t99-t237)+t344.*(t287-t5.*(t70+t3.*(t31-t40))).*3.0-t338.*(t294-t5.*(t83+t3.*(t31-t40)))+t334.*(t298-t5.*(t70+t3.*(t31-t40)).*(6.07e+2./2.0e+2))-t330.*(t310-t5.*(t83+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*3.0];
mt5 = [t217+t219+t220+t221+t350+t352+t354+t355+t358+t361+t364+t365+t375+t376+t377+t379+t393+t395-t303.*(t75-t228).*2.0+t273.*(t99-t237).*2.0+t338.*(t287-t5.*(t70+t3.*(t31-t40))).*2.0-t330.*(t298-t5.*(t70+t3.*(t31-t40)).*(6.07e+2./2.0e+2)).*2.0,et1+et2,et8+et9,et10+et11];
B = reshape([mt1,mt2,mt3,mt4,mt5],3,3);
