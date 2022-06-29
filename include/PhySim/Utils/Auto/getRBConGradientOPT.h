#include <math.h>

void getRBConGradientOPT(double young,
                         double shear,
                         double radw,
                         double radh,
                         double hL0,
                         const double* iniMatTanRB,
                         const double* iniMatNorRB,
                         const double* iniRefTanRO,
                         const double* iniRefNorRO,
                         double rtx,
                         const double* rotationAnglesRB,
                         const double* ex,
                         double thetax,
                         double* energyGradient) {
  double t10;
  double t100;
  double t1009;
  double t101;
  double t1013;
  double t1017;
  double t102;
  double t1021;
  double t1024;
  double t1029;
  double t1032;
  double t104;
  double t1042;
  double t1045;
  double t1049;
  double t105;
  double t1053;
  double t1055;
  double t1059;
  double t106;
  double t1064;
  double t1069;
  double t107;
  double t1078;
  double t108;
  double t1090;
  double t11;
  double t110;
  double t1104;
  double t1107;
  double t1118;
  double t112;
  double t1123;
  double t113;
  double t1136;
  double t1142;
  double t1155;
  double t116;
  double t1176;
  double t1179;
  double t118;
  double t1193;
  double t1198;
  double t12;
  double t1207;
  double t1216;
  double t122;
  double t1225;
  double t1238;
  double t125;
  double t128;
  double t1283;
  double t1284;
  double t1290;
  double t1296;
  double t1299;
  double t13;
  double t1303;
  double t1309;
  double t1311;
  double t1312;
  double t1313;
  double t1319;
  double t132;
  double t1323;
  double t1327;
  double t1329;
  double t133;
  double t1337;
  double t1342;
  double t1346;
  double t135;
  double t1354;
  double t1366;
  double t1367;
  double t1368;
  double t137;
  double t1375;
  double t1379;
  double t138;
  double t1382;
  double t1384;
  double t139;
  double t1393;
  double t14;
  double t1404;
  double t142;
  double t1422;
  double t1428;
  double t143;
  double t1434;
  double t144;
  double t145;
  double t1465;
  double t1467;
  double t147;
  double t1470;
  double t1477;
  double t1481;
  double t1483;
  double t1486;
  double t1489;
  double t149;
  double t1492;
  double t1494;
  double t1496;
  double t15;
  double t1502;
  double t1507;
  double t1513;
  double t1518;
  double t152;
  double t1524;
  double t1532;
  double t1537;
  double t154;
  double t1549;
  double t1562;
  double t1574;
  double t1577;
  double t158;
  double t1588;
  double t1599;
  double t16;
  double t161;
  double t1617;
  double t1623;
  double t1629;
  double t166;
  double t1661;
  double t1662;
  double t1665;
  double t1669;
  double t1671;
  double t1676;
  double t1678;
  double t1682;
  double t1687;
  double t1693;
  double t17;
  double t1705;
  double t1719;
  double t1731;
  double t1734;
  double t174;
  double t1745;
  double t175;
  double t1756;
  double t1774;
  double t178;
  double t1781;
  double t1788;
  double t18;
  double t1826;
  double t1829;
  double t1836;
  double t184;
  double t1846;
  double t189;
  double t19;
  double t199;
  double t20;
  double t205;
  double t21;
  double t215;
  double t217;
  double t22;
  double t221;
  double t222;
  double t223;
  double t225;
  double t227;
  double t228;
  double t229;
  double t23;
  double t231;
  double t232;
  double t235;
  double t236;
  double t237;
  double t239;
  double t24;
  double t240;
  double t241;
  double t243;
  double t244;
  double t247;
  double t249;
  double t250;
  double t254;
  double t256;
  double t257;
  double t26;
  double t262;
  double t265;
  double t266;
  double t269;
  double t271;
  double t273;
  double t277;
  double t28;
  double t280;
  double t281;
  double t283;
  double t29;
  double t291;
  double t294;
  double t296;
  double t299;
  double t3;
  double t300;
  double t301;
  double t302;
  double t303;
  double t305;
  double t306;
  double t310;
  double t312;
  double t313;
  double t316;
  double t317;
  double t319;
  double t320;
  double t323;
  double t325;
  double t326;
  double t331;
  double t333;
  double t334;
  double t335;
  double t339;
  double t341;
  double t343;
  double t35;
  double t351;
  double t354;
  double t356;
  double t359;
  double t364;
  double t368;
  double t369;
  double t373;
  double t377;
  double t379;
  double t381;
  double t385;
  double t391;
  double t392;
  double t394;
  double t396;
  double t4;
  double t404;
  double t410;
  double t413;
  double t416;
  double t418;
  double t422;
  double t424;
  double t43;
  double t431;
  double t433;
  double t434;
  double t439;
  double t443;
  double t446;
  double t45;
  double t452;
  double t456;
  double t46;
  double t463;
  double t465;
  double t466;
  double t469;
  double t47;
  double t471;
  double t472;
  double t473;
  double t48;
  double t484;
  double t487;
  double t49;
  double t495;
  double t496;
  double t498;
  double t5;
  double t50;
  double t506;
  double t507;
  double t508;
  double t511;
  double t518;
  double t519;
  double t52;
  double t521;
  double t524;
  double t529;
  double t53;
  double t530;
  double t534;
  double t537;
  double t542;
  double t547;
  double t55;
  double t551;
  double t556;
  double t562;
  double t566;
  double t570;
  double t574;
  double t578;
  double t579;
  double t58;
  double t584;
  double t590;
  double t591;
  double t597;
  double t6;
  double t601;
  double t604;
  double t61;
  double t614;
  double t615;
  double t62;
  double t621;
  double t627;
  double t63;
  double t631;
  double t637;
  double t64;
  double t65;
  double t67;
  double t68;
  double t69;
  double t691;
  double t696;
  double t698;
  double t699;
  double t7;
  double t702;
  double t705;
  double t708;
  double t709;
  double t71;
  double t715;
  double t718;
  double t72;
  double t723;
  double t728;
  double t736;
  double t739;
  double t74;
  double t744;
  double t745;
  double t748;
  double t749;
  double t753;
  double t756;
  double t759;
  double t764;
  double t769;
  double t78;
  double t780;
  double t785;
  double t793;
  double t8;
  double t80;
  double t806;
  double t81;
  double t810;
  double t813;
  double t817;
  double t825;
  double t83;
  double t830;
  double t835;
  double t84;
  double t846;
  double t854;
  double t86;
  double t867;
  double t869;
  double t870;
  double t88;
  double t89;
  double t891;
  double t9;
  double t90;
  double t906;
  double t91;
  double t910;
  double t914;
  double t915;
  double t918;
  double t927;
  double t93;
  double t936;
  double t941;
  double t946;
  double t95;
  double t957;
  double t96;
  double t960;
  double t968;
  double t98;
  t3 = shear * radw * radh * M_PI;
  t4 = radw * radw;
  t5 = radh * radh;
  t6 = t4 + t5;
  t15 = ex[0];
  t7 = t15 * t15;
  t18 = ex[1];
  t8 = t18 * t18;
  t21 = ex[2];
  t9 = t21 * t21;
  t10 = t7 + t8 + t9;
  t11 = sqrt(t10);
  t12 = 0.1e1 / t11;
  t13 = t12 * t18;
  t24 = iniRefTanRO[0];
  t14 = t24 * t12;
  t16 = 0.1e1 * t14 * t15;
  t26 = iniRefTanRO[1];
  t17 = t26 * t12;
  t19 = 0.1e1 * t17 * t18;
  t28 = iniRefTanRO[2];
  t20 = t28 * t12;
  t22 = 0.1e1 * t20 * t21;
  t23 = t16 + t19 + t22;
  t29 = 0.1e1 * t17 * t21 - 0.1e1 * t20 * t18;
  t35 = 0.1e1 * t20 * t15 - 0.1e1 * t14 * t21;
  t43 = 0.1e1 * t14 * t18 - 0.1e1 * t17 * t15;
  t49 = iniRefNorRO[0];
  t52 = iniRefNorRO[1];
  t55 = iniRefNorRO[2];
  t45 = t29 * t49 + t35 * t52 + t43 * t55;
  t46 = 0.1e1 + t16 + t19 + t22;
  t47 = 0.1e1 / t46;
  t48 = t45 * t47;
  t50 = t23 * t55 + t29 * t52 - t35 * t49 + t48 * t43;
  t53 = t12 * t21;
  t58 = t23 * t52 + t43 * t49 - t29 * t55 + t48 * t35;
  t61 = 0.1e1 * t13 * t50 - 0.1e1 * t53 * t58;
  t62 = cos(rtx);
  t88 = rotationAnglesRB[2];
  t63 = cos(t88);
  t90 = rotationAnglesRB[1];
  t64 = cos(t90);
  t65 = t63 * t64;
  t67 = sin(t90);
  t68 = t63 * t67;
  t91 = rotationAnglesRB[0];
  t69 = sin(t91);
  t71 = sin(t88);
  t72 = cos(t91);
  t74 = t68 * t69 - t71 * t72;
  t78 = t68 * t72 + t71 * t69;
  t101 = iniMatTanRB[0];
  t106 = iniMatTanRB[1];
  t110 = iniMatTanRB[2];
  t80 = t65 * t101 + t74 * t106 + t78 * t110;
  t81 = t80 * t12;
  t83 = 0.1e1 * t81 * t15;
  t84 = t71 * t64;
  t86 = t71 * t67;
  t89 = t86 * t69 + t63 * t72;
  t93 = t86 * t72 - t63 * t69;
  t95 = t84 * t101 + t89 * t106 + t93 * t110;
  t96 = t95 * t12;
  t98 = 0.1e1 * t96 * t18;
  t100 = t64 * t69;
  t102 = t64 * t72;
  t104 = -t67 * t101 + t100 * t106 + t102 * t110;
  t105 = t104 * t12;
  t107 = 0.1e1 * t105 * t21;
  t108 = t83 + t98 + t107;
  t133 = iniMatNorRB[0];
  t135 = iniMatNorRB[1];
  t137 = iniMatNorRB[2];
  t112 = t65 * t133 + t74 * t135 + t78 * t137;
  t113 = t108 * t112;
  t116 = t81 * t21;
  t118 = 0.1e1 * t105 * t15 - 0.1e1 * t116;
  t122 = -t67 * t133 + t100 * t135 + t102 * t137;
  t125 = 0.1e1 * t81 * t18;
  t128 = t125 - 0.1e1 * t96 * t15;
  t132 = t84 * t133 + t89 * t135 + t93 * t137;
  t138 = 0.1e1 * t96 * t21 - 0.1e1 * t105 * t18;
  t139 = t138 * t112;
  t142 = t139 + t118 * t132 + t128 * t122;
  t143 = 0.1e1 + t83 + t98 + t107;
  t144 = 0.1e1 / t143;
  t145 = t142 * t144;
  t147 = t113 + t118 * t122 - t128 * t132 + t145 * t138;
  t149 = sin(rtx);
  t152 = t118 * t112;
  t154 = t108 * t122 + t138 * t132 - t152 + t145 * t128;
  t158 = t128 * t112;
  t161 = t108 * t132 + t158 - t138 * t122 + t145 * t118;
  t166 = t12 * t15;
  t174 = 0.1e1 - t62;
  t175 = (0.1e1 * t166 * t147 + 0.1e1 * t13 * t161 + 0.1e1 * t53 * t154) * t174;
  t178 = t62 * t147 + t149 * (0.1e1 * t13 * t154 - 0.1e1 * t53 * t161) +
         0.1e1 * t175 * t166;
  t184 = t23 * t49 + t35 * t55 - t43 * t52 + t48 * t29;
  t189 = 0.1e1 * t53 * t184 - 0.1e1 * t166 * t50;
  t199 = t62 * t161 + t149 * (0.1e1 * t53 * t147 - 0.1e1 * t166 * t154) +
         0.1e1 * t175 * t13;
  t205 = 0.1e1 * t166 * t58 - 0.1e1 * t13 * t184;
  t215 = t62 * t154 + t149 * (0.1e1 * t166 * t161 - 0.1e1 * t13 * t147) +
         0.1e1 * t175 * t53;
  t217 = t61 * t178 + t189 * t199 + t205 * t215;
  t221 = t184 * t178 + t58 * t199 + t50 * t215;
  t222 = atan2(t217, t221);
  t223 = thetax + rtx - t222;
  t225 = 0.1e1 / hL0;
  t227 = t12 / t10;
  t228 = t227 * t18;
  t229 = t50 * t15;
  t231 = 0.1e1 * t228 * t229;
  t232 = t24 * t227;
  t235 = 0.1e1 * t14;
  t236 = t26 * t227;
  t237 = t18 * t15;
  t239 = 0.1e1 * t236 * t237;
  t240 = t28 * t227;
  t241 = t21 * t15;
  t243 = 0.1e1 * t240 * t241;
  t244 = -0.1e1 * t232 * t7 + t235 - t239 - t243;
  t247 = 0.1e1 * t236 * t241;
  t249 = 0.1e1 * t240 * t237;
  t250 = -t247 + t249;
  t254 = 0.1e1 * t20;
  t256 = 0.1e1 * t232 * t241;
  t257 = -0.1e1 * t240 * t7 + t254 + t256;
  t262 = 0.1e1 * t232 * t237;
  t265 = 0.1e1 * t17;
  t266 = -t262 + 0.1e1 * t236 * t7 - t265;
  t269 = (t250 * t49 + t257 * t52 + t266 * t55) * t47;
  t271 = t46 * t46;
  t273 = t45 / t271;
  t301 = t273 * t43;
  t277 = t244 * t55 + t250 * t52 - t257 * t49 + t269 * t43 - t301 * t244 +
         t48 * t266;
  t280 = t227 * t21;
  t281 = t58 * t15;
  t283 = 0.1e1 * t280 * t281;
  t317 = t273 * t35;
  t291 = t244 * t52 + t266 * t49 - t250 * t55 + t269 * t35 - t317 * t244 +
         t48 * t257;
  t294 = -t231 + 0.1e1 * t13 * t277 + t283 - 0.1e1 * t53 * t291;
  t296 = t80 * t227;
  t299 = 0.1e1 * t81;
  t300 = t95 * t227;
  t302 = 0.1e1 * t300 * t237;
  t303 = t104 * t227;
  t305 = 0.1e1 * t303 * t241;
  t306 = -0.1e1 * t296 * t7 + t299 - t302 - t305;
  t310 = 0.1e1 * t105;
  t312 = 0.1e1 * t296 * t241;
  t313 = -0.1e1 * t303 * t7 + t310 + t312;
  t316 = 0.1e1 * t296 * t237;
  t319 = 0.1e1 * t96;
  t320 = -t316 + 0.1e1 * t300 * t7 - t319;
  t323 = 0.1e1 * t300 * t241;
  t325 = 0.1e1 * t237 * t303;
  t326 = -t323 + t325;
  t331 = (t326 * t112 + t313 * t132 + t320 * t122) * t144;
  t333 = t143 * t143;
  t334 = 0.1e1 / t333;
  t335 = t142 * t334;
  t359 = t335 * t138;
  t339 = t306 * t112 + t313 * t122 - t320 * t132 + t331 * t138 - t359 * t306 +
         t145 * t326;
  t341 = t154 * t15;
  t343 = 0.1e1 * t228 * t341;
  t368 = t335 * t128;
  t351 = t306 * t122 + t326 * t132 - t313 * t112 + t331 * t128 - t368 * t306 +
         t145 * t320;
  t354 = t161 * t15;
  t356 = 0.1e1 * t280 * t354;
  t379 = t335 * t118;
  t364 = t306 * t132 + t320 * t112 - t326 * t122 + t331 * t118 - t379 * t306 +
         t145 * t313;
  t369 = t227 * t7;
  t373 = 0.1e1 * t12 * t147;
  t377 = 0.1e1 * t228 * t354;
  t381 = 0.1e1 * t280 * t341;
  t385 = (-0.1e1 * t369 * t147 + t373 + 0.1e1 * t166 * t339 - t377 +
          0.1e1 * t13 * t364 - t381 + 0.1e1 * t53 * t351) *
         t174;
  t391 = 0.1e1 * t175 * t12;
  t392 = t62 * t339 +
         t149 * (-t343 + 0.1e1 * t13 * t351 + t356 - 0.1e1 * t53 * t364) +
         0.1e1 * t385 * t166 - 0.1e1 * t175 * t369 + t391;
  t394 = t184 * t15;
  t396 = 0.1e1 * t280 * t394;
  t422 = t273 * t29;
  t404 = t244 * t49 + t257 * t55 - t266 * t52 + t269 * t29 - t422 * t244 +
         t48 * t250;
  t410 = 0.1e1 * t12 * t50;
  t413 = -t396 + 0.1e1 * t53 * t404 + 0.1e1 * t369 * t50 - t410 -
         0.1e1 * t166 * t277;
  t416 = t147 * t15;
  t418 = 0.1e1 * t280 * t416;
  t424 = 0.1e1 * t12 * t154;
  t431 = t228 * t15;
  t433 = 0.1e1 * t175 * t431;
  t434 = t62 * t364 +
         t149 * (-t418 + 0.1e1 * t53 * t339 + 0.1e1 * t369 * t154 - t424 -
                 0.1e1 * t166 * t351) +
         0.1e1 * t385 * t13 - t433;
  t439 = 0.1e1 * t12 * t58;
  t443 = 0.1e1 * t228 * t394;
  t446 = -0.1e1 * t369 * t58 + t439 + 0.1e1 * t166 * t291 + t443 -
         0.1e1 * t13 * t404;
  t452 = 0.1e1 * t12 * t161;
  t456 = 0.1e1 * t228 * t416;
  t463 = t280 * t15;
  t465 = 0.1e1 * t175 * t463;
  t466 = t62 * t351 +
         t149 * (-0.1e1 * t369 * t161 + t452 + 0.1e1 * t166 * t364 + t456 -
                 0.1e1 * t13 * t339) +
         0.1e1 * t385 * t53 - t465;
  t469 = 0.1e1 / t221;
  t471 = t221 * t221;
  t472 = 0.1e1 / t471;
  t473 = t217 * t472;
  t484 = t217 * t217;
  t487 = 0.1e1 / (0.1e1 + t484 * t472);
  t495 = t144 * t138;
  t496 = cos(thetax);
  t498 = sin(thetax);
  t506 = 0.1e1 * t166 * t184 + 0.1e1 * t13 * t58 + 0.1e1 * t53 * t50;
  t507 = 0.1e1 - t496;
  t508 = t506 * t507;
  t511 = t496 * t50 + t498 * t205 + 0.1e1 * t508 * t53;
  t518 = t496 * t58 + t498 * t189 + 0.1e1 * t508 * t13;
  t519 = t53 * t518;
  t521 = 0.1e1 * t13 * t511 - 0.1e1 * t519;
  t524 = t144 * t118;
  t529 = t496 * t184 + t498 * t61 + 0.1e1 * t508 * t166;
  t530 = t53 * t529;
  t534 = 0.1e1 * t530 - 0.1e1 * t166 * t511;
  t537 = t144 * t128;
  t542 = 0.1e1 * t166 * t518 - 0.1e1 * t13 * t529;
  t547 = t334 * t138;
  t551 = t144 * t326;
  t556 = 0.1e1 * t228 * t511 * t15;
  t562 = 0.1e1 * t12 * t184;
  t566 = 0.1e1 * t228 * t281;
  t570 = 0.1e1 * t280 * t229;
  t574 = (-0.1e1 * t369 * t184 + t562 + 0.1e1 * t166 * t404 - t566 +
          0.1e1 * t13 * t291 - t570 + 0.1e1 * t53 * t277) *
         t507;
  t578 = 0.1e1 * t508 * t463;
  t579 = t496 * t277 + t498 * t446 + 0.1e1 * t574 * t53 - t578;
  t584 = 0.1e1 * t280 * t518 * t15;
  t590 = 0.1e1 * t508 * t431;
  t591 = t496 * t291 + t498 * t413 + 0.1e1 * t574 * t13 - t590;
  t597 = t334 * t118;
  t601 = t144 * t313;
  t604 = t529 * t15;
  t614 = 0.1e1 * t508 * t12;
  t615 = t496 * t404 + t498 * t294 + 0.1e1 * t574 * t166 - 0.1e1 * t508 * t369 +
         t614;
  t621 = 0.1e1 * t12 * t511;
  t627 = t334 * t128;
  t631 = t144 * t320;
  t637 = 0.1e1 * t12 * t518;
  t691 = t227 * t8;
  t696 = t21 * t18;
  t698 = 0.1e1 * t240 * t696;
  t699 = -t262 - 0.1e1 * t236 * t8 + t265 - t698;
  t702 = 0.1e1 * t236 * t696;
  t705 = -t702 + 0.1e1 * t240 * t8 - t254;
  t708 = 0.1e1 * t232 * t696;
  t709 = -t249 + t708;
  t715 = -0.1e1 * t232 * t8 + t235 + t239;
  t718 = (t705 * t49 + t709 * t52 + t715 * t55) * t47;
  t723 = t699 * t55 + t705 * t52 - t709 * t49 + t718 * t43 - t301 * t699 +
         t48 * t715;
  t728 = 0.1e1 * t280 * t58 * t18;
  t736 = t699 * t52 + t715 * t49 - t705 * t55 + t718 * t35 - t317 * t699 +
         t48 * t709;
  t739 = -0.1e1 * t691 * t50 + t410 + 0.1e1 * t13 * t723 + t728 -
         0.1e1 * t53 * t736;
  t744 = 0.1e1 * t303 * t696;
  t745 = -t316 - 0.1e1 * t300 * t8 + t319 - t744;
  t748 = 0.1e1 * t296 * t696;
  t749 = -t325 + t748;
  t753 = -0.1e1 * t296 * t8 + t299 + t302;
  t756 = 0.1e1 * t300 * t696;
  t759 = -t756 + 0.1e1 * t303 * t8 - t310;
  t764 = (t759 * t112 + t749 * t132 + t753 * t122) * t144;
  t769 = t745 * t112 + t749 * t122 - t753 * t132 + t764 * t138 - t359 * t745 +
         t145 * t759;
  t780 = t745 * t122 + t759 * t132 - t749 * t112 + t764 * t128 - t368 * t745 +
         t145 * t753;
  t785 = 0.1e1 * t280 * t161 * t18;
  t793 = t745 * t132 + t753 * t112 - t759 * t122 + t764 * t118 - t379 * t745 +
         t145 * t749;
  t806 = 0.1e1 * t280 * t154 * t18;
  t810 = (-t456 + 0.1e1 * t166 * t769 - 0.1e1 * t691 * t161 + t452 +
          0.1e1 * t13 * t793 - t806 + 0.1e1 * t53 * t780) *
         t174;
  t813 = t62 * t769 +
         t149 * (-0.1e1 * t691 * t154 + t424 + 0.1e1 * t13 * t780 + t785 -
                 0.1e1 * t53 * t793) +
         0.1e1 * t810 * t166 - t433;
  t817 = 0.1e1 * t280 * t184 * t18;
  t825 = t699 * t49 + t709 * t55 - t715 * t52 + t718 * t29 - t422 * t699 +
         t48 * t705;
  t830 = -t817 + 0.1e1 * t53 * t825 + t231 - 0.1e1 * t166 * t723;
  t835 = 0.1e1 * t280 * t147 * t18;
  t846 = t62 * t793 +
         t149 * (-t835 + 0.1e1 * t53 * t769 + t343 - 0.1e1 * t166 * t780) +
         0.1e1 * t810 * t13 - 0.1e1 * t175 * t691 + t391;
  t854 = -t566 + 0.1e1 * t166 * t736 + 0.1e1 * t691 * t184 - t562 -
         0.1e1 * t13 * t825;
  t867 = t280 * t18;
  t869 = 0.1e1 * t175 * t867;
  t870 = t62 * t780 +
         t149 * (-t377 + 0.1e1 * t166 * t793 + 0.1e1 * t691 * t147 - t373 -
                 0.1e1 * t13 * t769) +
         0.1e1 * t810 * t53 - t869;
  t891 = t144 * t759;
  t906 = 0.1e1 * t280 * t50 * t18;
  t910 = (-t443 + 0.1e1 * t166 * t825 - 0.1e1 * t691 * t58 + t439 +
          0.1e1 * t13 * t736 - t906 + 0.1e1 * t53 * t723) *
         t507;
  t914 = 0.1e1 * t508 * t867;
  t915 = t496 * t723 + t498 * t854 + 0.1e1 * t910 * t53 - t914;
  t918 = t518 * t18;
  t927 = t496 * t736 + t498 * t830 + 0.1e1 * t910 * t13 - 0.1e1 * t508 * t691 +
         t614;
  t936 = t144 * t749;
  t941 = 0.1e1 * t280 * t529 * t18;
  t946 = t496 * t825 + t498 * t739 + 0.1e1 * t910 * t166 - t590;
  t957 = t144 * t753;
  t960 = t227 * t15;
  t968 = 0.1e1 * t12 * t529;
  t1009 = -t256 - t702 - 0.1e1 * t240 * t9 + t254;
  t1013 = -0.1e1 * t236 * t9 + t265 + t698;
  t1017 = -t243 + 0.1e1 * t232 * t9 - t235;
  t1021 = -t708 + t247;
  t1024 = (t1013 * t49 + t1017 * t52 + t1021 * t55) * t47;
  t1029 = t1009 * t55 + t1013 * t52 - t1017 * t49 + t1024 * t43 - t301 * t1009 +
          t48 * t1021;
  t1032 = t227 * t9;
  t1042 = t1009 * t52 + t1021 * t49 - t1013 * t55 + t1024 * t35 - t317 * t1009 +
          t48 * t1017;
  t1045 = -t906 + 0.1e1 * t13 * t1029 + 0.1e1 * t1032 * t58 - t439 -
          0.1e1 * t53 * t1042;
  t1049 = -t312 - t756 - 0.1e1 * t303 * t9 + t310;
  t1053 = -t305 + 0.1e1 * t296 * t9 - t299;
  t1055 = -t748 + t323;
  t1059 = -0.1e1 * t300 * t9 + t319 + t744;
  t1064 = (t1059 * t112 + t1053 * t132 + t1055 * t122) * t144;
  t1069 = t1049 * t112 + t1053 * t122 - t1055 * t132 + t1064 * t138 -
          t359 * t1049 + t145 * t1059;
  t1078 = t1049 * t122 + t1059 * t132 - t1053 * t112 + t1064 * t128 -
          t368 * t1049 + t145 * t1055;
  t1090 = t1049 * t132 + t1055 * t112 - t1059 * t122 + t1064 * t118 -
          t379 * t1049 + t145 * t1053;
  t1104 = (-t418 + 0.1e1 * t166 * t1069 - t785 + 0.1e1 * t13 * t1090 -
           0.1e1 * t1032 * t154 + t424 + 0.1e1 * t53 * t1078) *
          t174;
  t1107 = t62 * t1069 +
          t149 * (-t806 + 0.1e1 * t13 * t1078 + 0.1e1 * t1032 * t161 - t452 -
                  0.1e1 * t53 * t1090) +
          0.1e1 * t1104 * t166 - t465;
  t1118 = t1009 * t49 + t1017 * t55 - t1021 * t52 + t1024 * t29 - t422 * t1009 +
          t48 * t1013;
  t1123 = -0.1e1 * t1032 * t184 + t562 + 0.1e1 * t53 * t1118 + t570 -
          0.1e1 * t166 * t1029;
  t1136 = t62 * t1090 +
          t149 * (-0.1e1 * t1032 * t147 + t373 + 0.1e1 * t53 * t1069 + t381 -
                  0.1e1 * t166 * t1078) +
          0.1e1 * t1104 * t13 - t869;
  t1142 = -t283 + 0.1e1 * t166 * t1042 + t817 - 0.1e1 * t13 * t1118;
  t1155 = t62 * t1078 +
          t149 * (-t356 + 0.1e1 * t166 * t1090 + t835 - 0.1e1 * t13 * t1069) +
          0.1e1 * t1104 * t53 - 0.1e1 * t175 * t1032 + t391;
  t1176 = t144 * t1059;
  t1179 = t511 * t21;
  t1193 = (-t396 + 0.1e1 * t166 * t1118 - t728 + 0.1e1 * t13 * t1042 -
           0.1e1 * t1032 * t50 + t410 + 0.1e1 * t53 * t1029) *
          t507;
  t1198 = t496 * t1029 + t498 * t1142 + 0.1e1 * t1193 * t53 -
          0.1e1 * t508 * t1032 + t614;
  t1207 = t496 * t1042 + t498 * t1123 + 0.1e1 * t1193 * t13 - t914;
  t1216 = t144 * t1053;
  t1225 = t496 * t1118 + t498 * t1045 + 0.1e1 * t1193 * t166 - t578;
  t1238 = t144 * t1055;
  t1284 = (t78 * t106 - t74 * t110) * t12;
  t1290 = (t93 * t106 - t89 * t110) * t12;
  t1296 = (t102 * t106 - t100 * t110) * t12;
  t1299 = 0.1e1 * t1284 * t15 + 0.1e1 * t1290 * t18 + 0.1e1 * t1296 * t21;
  t1303 = t78 * t135 - t74 * t137;
  t1309 = 0.1e1 * t1296 * t15 - 0.1e1 * t1284 * t21;
  t1313 = t102 * t135 - t100 * t137;
  t1319 = 0.1e1 * t1284 * t18 - 0.1e1 * t1290 * t15;
  t1323 = t93 * t135 - t89 * t137;
  t1329 = 0.1e1 * t1290 * t21 - 0.1e1 * t1296 * t18;
  t1337 = (t1329 * t112 + t138 * t1303 + t1309 * t132 + t118 * t1323 +
           t1319 * t122 + t128 * t1313) *
          t144;
  t1342 = t1299 * t112 + t108 * t1303 + t1309 * t122 + t118 * t1313 -
          t1319 * t132 - t128 * t1323 + t1337 * t138 - t359 * t1299 +
          t145 * t1329;
  t1354 = t1299 * t122 + t108 * t1313 + t1329 * t132 + t138 * t1323 -
          t1309 * t112 - t118 * t1303 + t1337 * t128 - t368 * t1299 +
          t145 * t1319;
  t1367 = t1299 * t132 + t108 * t1323 + t1319 * t112 + t128 * t1303 -
          t1329 * t122 - t138 * t1313 + t1337 * t118 - t379 * t1299 +
          t145 * t1309;
  t1379 =
      (0.1e1 * t166 * t1342 + 0.1e1 * t13 * t1367 + 0.1e1 * t53 * t1354) * t174;
  t1382 = t62 * t1342 + t149 * (0.1e1 * t13 * t1354 - 0.1e1 * t53 * t1367) +
          0.1e1 * t1379 * t166;
  t1393 = t62 * t1367 + t149 * (0.1e1 * t53 * t1342 - 0.1e1 * t166 * t1354) +
          0.1e1 * t1379 * t13;
  t1404 = t62 * t1354 + t149 * (0.1e1 * t166 * t1367 - 0.1e1 * t13 * t1342) +
          0.1e1 * t1379 * t53;
  t1422 = t144 * t1329;
  t1428 = t144 * t1309;
  t1434 = t144 * t1319;
  t1465 = t69 * t106;
  t1467 = t72 * t110;
  t1470 = (-t68 * t101 + t65 * t1465 + t65 * t1467) * t12;
  t1477 = (-t86 * t101 + t84 * t1465 + t84 * t1467) * t12;
  t1481 = t67 * t69;
  t1483 = t67 * t72;
  t1486 = (-t64 * t101 - t1481 * t106 - t1483 * t110) * t12;
  t1489 = 0.1e1 * t1470 * t15 + 0.1e1 * t1477 * t18 + 0.1e1 * t1486 * t21;
  t1492 = t69 * t135;
  t1494 = t72 * t137;
  t1496 = -t68 * t133 + t65 * t1492 + t65 * t1494;
  t1502 = 0.1e1 * t1486 * t15 - 0.1e1 * t1470 * t21;
  t1507 = -t64 * t133 - t1481 * t135 - t1483 * t137;
  t1513 = 0.1e1 * t1470 * t18 - 0.1e1 * t1477 * t15;
  t1518 = -t86 * t133 + t84 * t1492 + t84 * t1494;
  t1524 = 0.1e1 * t1477 * t21 - 0.1e1 * t1486 * t18;
  t1532 = (t1524 * t112 + t138 * t1496 + t1502 * t132 + t118 * t1518 +
           t1513 * t122 + t128 * t1507) *
          t144;
  t1537 = t1489 * t112 + t108 * t1496 + t1502 * t122 + t118 * t1507 -
          t1513 * t132 - t128 * t1518 + t1532 * t138 - t359 * t1489 +
          t145 * t1524;
  t1549 = t1489 * t122 + t108 * t1507 + t1524 * t132 + t138 * t1518 -
          t1502 * t112 - t118 * t1496 + t1532 * t128 - t368 * t1489 +
          t145 * t1513;
  t1562 = t1489 * t132 + t108 * t1518 + t1513 * t112 + t128 * t1496 -
          t1524 * t122 - t138 * t1507 + t1532 * t118 - t379 * t1489 +
          t145 * t1502;
  t1574 =
      (0.1e1 * t166 * t1537 + 0.1e1 * t13 * t1562 + 0.1e1 * t53 * t1549) * t174;
  t1577 = t62 * t1537 + t149 * (0.1e1 * t13 * t1549 - 0.1e1 * t53 * t1562) +
          0.1e1 * t1574 * t166;
  t1588 = t62 * t1562 + t149 * (0.1e1 * t53 * t1537 - 0.1e1 * t166 * t1549) +
          0.1e1 * t1574 * t13;
  t1599 = t62 * t1549 + t149 * (0.1e1 * t166 * t1562 - 0.1e1 * t13 * t1537) +
          0.1e1 * t1574 * t53;
  t1617 = t144 * t1524;
  t1623 = t144 * t1502;
  t1629 = t144 * t1513;
  t1661 = -t95;
  t1662 = t1661 * t12;
  t1665 = 0.1e1 * t1662 * t15 + t125;
  t1669 = -t132;
  t1671 = t21 * t122;
  t1676 = 0.1e1 * t1662 * t18 - t83;
  t1678 = t21 * t112;
  t1682 = t21 * t132;
  t1687 = (0.1e1 * t81 * t1678 + t138 * t1669 - 0.1e1 * t1662 * t1682 + t152 +
           t1676 * t122) *
          t144;
  t1693 = t1665 * t112 + t108 * t1669 - 0.1e1 * t1662 * t1671 - t1676 * t132 -
          t158 + t1687 * t138 - t359 * t1665 + 0.1e1 * t145 * t116;
  t1705 = t1665 * t122 + 0.1e1 * t81 * t1682 + t139 + 0.1e1 * t1662 * t1678 -
          t118 * t1669 + t1687 * t128 - t368 * t1665 + t145 * t1676;
  t1719 = t1665 * t132 + t113 + t1676 * t112 + t128 * t1669 -
          0.1e1 * t81 * t1671 + t1687 * t118 - t379 * t1665 -
          0.1e1 * t145 * t1662 * t21;
  t1731 =
      (0.1e1 * t166 * t1693 + 0.1e1 * t13 * t1719 + 0.1e1 * t53 * t1705) * t174;
  t1734 = t62 * t1693 + t149 * (0.1e1 * t13 * t1705 - 0.1e1 * t53 * t1719) +
          0.1e1 * t1731 * t166;
  t1745 = t62 * t1719 + t149 * (0.1e1 * t53 * t1693 - 0.1e1 * t166 * t1705) +
          0.1e1 * t1731 * t13;
  t1756 = t62 * t1705 + t149 * (0.1e1 * t166 * t1719 - 0.1e1 * t13 * t1693) +
          0.1e1 * t1731 * t53;
  t1774 = t144 * t80;
  t1781 = t144 * t1661;
  t1788 = t144 * t1676;
  t1826 = t506 * t498;
  t1829 = -t498 * t50 + t496 * t205 + 0.1e1 * t1826 * t53;
  t1836 = -t498 * t58 + t496 * t189 + 0.1e1 * t1826 * t13;
  t1846 = -t498 * t184 + t496 * t61 + 0.1e1 * t1826 * t166;
  t1283 = t3 * t6 * t223;
  t1311 = young * radw * t5 * radh * M_PI *
          (0.2e1 * t495 * t521 + 0.2e1 * t524 * t534 + 0.2e1 * t537 * t542);
  t1312 = t547 * t521;
  t1327 = t597 * t534;
  t1346 = t627 * t542;
  t1366 = young * t4 * radw * radh * M_PI *
          (-0.2e1 * t495 * t529 - 0.2e1 * t524 * t518 - 0.2e1 * t537 * t511);
  t1368 = t547 * t529;
  t1375 = t597 * t518;
  t1384 = t627 * t511;
  energyGradient[0] =
      -0.2500000000e0 * t1283 * t225 *
          ((t294 * t178 + t61 * t392 + t413 * t199 + t189 * t434 + t446 * t215 +
            t205 * t466) *
               t469 -
           t473 * (t178 * t404 + t184 * t392 + t291 * t199 + t58 * t434 +
                   t277 * t215 + t50 * t466)) *
          t487 +
      (0.50e0 * t1311 *
           (-0.2e1 * t1312 * t306 + 0.2e1 * t551 * t521 +
            0.2e1 * t495 *
                (-t556 + 0.1e1 * t13 * t579 + t584 - 0.1e1 * t53 * t591) -
            0.2e1 * t1327 * t306 + 0.2e1 * t601 * t534 +
            0.2e1 * t524 *
                (-0.1e1 * t280 * t604 + 0.1e1 * t53 * t615 +
                 0.1e1 * t369 * t511 - t621 - 0.1e1 * t166 * t579) -
            0.2e1 * t1346 * t306 + 0.2e1 * t631 * t542 +
            0.2e1 * t537 *
                (-0.1e1 * t369 * t518 + t637 + 0.1e1 * t166 * t591 +
                 0.1e1 * t228 * t604 - 0.1e1 * t13 * t615)) +
       0.50e0 * t1366 *
           (0.2e1 * t1368 * t306 - 0.2e1 * t551 * t529 - 0.2e1 * t495 * t615 +
            0.2e1 * t1375 * t306 - 0.2e1 * t601 * t518 - 0.2e1 * t524 * t591 +
            0.2e1 * t1384 * t306 - 0.2e1 * t631 * t511 - 0.2e1 * t537 * t579)) *
          t225 / 0.2e1;
  energyGradient[1] =
      -0.2500000000e0 * t1283 * t225 *
          ((t739 * t178 + t61 * t813 + t830 * t199 + t189 * t846 + t854 * t215 +
            t205 * t870) *
               t469 -
           t473 * (t825 * t178 + t184 * t813 + t736 * t199 + t58 * t846 +
                   t723 * t215 + t50 * t870)) *
          t487 +
      (0.50e0 * t1311 *
           (-0.2e1 * t1312 * t745 + 0.2e1 * t891 * t521 +
            0.2e1 * t495 *
                (-0.1e1 * t691 * t511 + t621 + 0.1e1 * t13 * t915 +
                 0.1e1 * t280 * t918 - 0.1e1 * t53 * t927) -
            0.2e1 * t1327 * t745 + 0.2e1 * t936 * t534 +
            0.2e1 * t524 *
                (-t941 + 0.1e1 * t53 * t946 + t556 - 0.1e1 * t166 * t915) -
            0.2e1 * t1346 * t745 + 0.2e1 * t957 * t542 +
            0.2e1 * t537 *
                (-0.1e1 * t960 * t918 + 0.1e1 * t166 * t927 +
                 0.1e1 * t691 * t529 - t968 - 0.1e1 * t13 * t946)) +
       0.50e0 * t1366 *
           (0.2e1 * t1368 * t745 - 0.2e1 * t891 * t529 - 0.2e1 * t495 * t946 +
            0.2e1 * t1375 * t745 - 0.2e1 * t936 * t518 - 0.2e1 * t524 * t927 +
            0.2e1 * t745 * t1384 - 0.2e1 * t957 * t511 - 0.2e1 * t537 * t915)) *
          t225 / 0.2e1;
  energyGradient[2] =
      -0.2500000000e0 * t1283 * t225 *
          ((t1045 * t178 + t61 * t1107 + t1123 * t199 + t189 * t1136 +
            t1142 * t215 + t205 * t1155) *
               t469 -
           t473 * (t1118 * t178 + t184 * t1107 + t1042 * t199 + t58 * t1136 +
                   t1029 * t215 + t50 * t1155)) *
          t487 +
      (0.50e0 * t1311 *
           (-0.2e1 * t1312 * t1049 + 0.2e1 * t1176 * t521 +
            0.2e1 * t495 *
                (-0.1e1 * t228 * t1179 + 0.1e1 * t13 * t1198 +
                 0.1e1 * t1032 * t518 - t637 - 0.1e1 * t53 * t1207) -
            0.2e1 * t1327 * t1049 + 0.2e1 * t1216 * t534 +
            0.2e1 * t524 *
                (-0.1e1 * t1032 * t529 + t968 + 0.1e1 * t53 * t1225 +
                 0.1e1 * t960 * t1179 - 0.1e1 * t166 * t1198) -
            0.2e1 * t1346 * t1049 + 0.2e1 * t1238 * t542 +
            0.2e1 * t537 *
                (-t584 + 0.1e1 * t166 * t1207 + t941 - 0.1e1 * t13 * t1225)) +
       0.50e0 * t1366 *
           (0.2e1 * t1368 * t1049 - 0.2e1 * t1176 * t529 -
            0.2e1 * t495 * t1225 + 0.2e1 * t1375 * t1049 -
            0.2e1 * t1216 * t518 - 0.2e1 * t524 * t1207 +
            0.2e1 * t1384 * t1049 - 0.2e1 * t1238 * t511 -
            0.2e1 * t537 * t1198)) *
          t225 / 0.2e1;
  energyGradient[3] = -0.2500000000e0 * t1283 * t225 *
                          ((t61 * t1382 + t189 * t1393 + t205 * t1404) * t469 -
                           t473 * (t184 * t1382 + t58 * t1393 + t50 * t1404)) *
                          t487 +
                      (0.50e0 * t1311 *
                           (-0.2e1 * t1312 * t1299 + 0.2e1 * t1422 * t521 -
                            0.2e1 * t1327 * t1299 + 0.2e1 * t1428 * t534 -
                            0.2e1 * t1346 * t1299 + 0.2e1 * t1434 * t542) +
                       0.50e0 * t1366 *
                           (0.2e1 * t1368 * t1299 - 0.2e1 * t1422 * t529 +
                            0.2e1 * t1375 * t1299 - 0.2e1 * t1428 * t518 +
                            0.2e1 * t1384 * t1299 - 0.2e1 * t1434 * t511)) *
                          t225 / 0.2e1;
  energyGradient[4] = -0.2500000000e0 * t1283 * t225 *
                          ((t61 * t1577 + t189 * t1588 + t205 * t1599) * t469 -
                           t473 * (t184 * t1577 + t58 * t1588 + t50 * t1599)) *
                          t487 +
                      (0.50e0 * t1311 *
                           (-0.2e1 * t1312 * t1489 + 0.2e1 * t1617 * t521 -
                            0.2e1 * t1327 * t1489 + 0.2e1 * t1623 * t534 -
                            0.2e1 * t1346 * t1489 + 0.2e1 * t1629 * t542) +
                       0.50e0 * t1366 *
                           (0.2e1 * t1368 * t1489 - 0.2e1 * t1617 * t529 +
                            0.2e1 * t1375 * t1489 - 0.2e1 * t1623 * t518 +
                            0.2e1 * t1384 * t1489 - 0.2e1 * t1629 * t511)) *
                          t225 / 0.2e1;
  energyGradient[5] =
      -0.2500000000e0 * t1283 * t225 *
          ((t61 * t1734 + t189 * t1745 + t205 * t1756) * t469 -
           t473 * (t184 * t1734 + t58 * t1745 + t50 * t1756)) *
          t487 +
      (0.50e0 * t1311 *
           (-0.2e1 * t1312 * t1665 + 0.2e1 * t1774 * t53 * t521 -
            0.2e1 * t1327 * t1665 - 0.2e1 * t1781 * t53 * t534 -
            0.2e1 * t1346 * t1665 + 0.2e1 * t1788 * t542) +
       0.50e0 * t1366 *
           (0.2e1 * t1368 * t1665 - 0.2e1 * t1774 * t530 +
            0.2e1 * t1375 * t1665 + 0.2e1 * t1781 * t519 +
            0.2e1 * t1384 * t1665 - 0.2e1 * t1788 * t511)) *
          t225 / 0.2e1;
  energyGradient[6] =
      0.2500000000e0 * t3 * t6 * t223 * t225 +
      (0.50e0 * t1311 *
           (0.2e1 * t495 * (0.1e1 * t13 * t1829 - 0.1e1 * t53 * t1836) +
            0.2e1 * t524 * (0.1e1 * t53 * t1846 - 0.1e1 * t166 * t1829) +
            0.2e1 * t537 * (0.1e1 * t166 * t1836 - 0.1e1 * t13 * t1846)) +
       0.50e0 * t1366 *
           (-0.2e1 * t495 * t1846 - 0.2e1 * t524 * t1836 -
            0.2e1 * t537 * t1829)) *
          t225 / 0.2e1;
}
