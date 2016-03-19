MODULE cc_11;
 TYPE GENERAL;
 DIMENSIONS 3037 0 3037 1826 -109 1826 -109 0;
 IOLIST;
  P_2 B 10 1826 1 METAL2;
  P_3 B 2 1826 1 METAL2;
  P_4 B 18 1826 1 METAL2;
  P_5 B 3037 1638 1 METAL2;
  P_6 B 3037 1319 1 METAL2;
  P_7 B 3037 1159 1 METAL2;
  P_8 B 3037 967 1 METAL2;
 ENDIOLIST;
ENDMODULE;
MODULE cc_12;
 TYPE GENERAL;
 DIMENSIONS 3037 0 3037 1826 -109 1826 -109 0;
 IOLIST;
  P_2 B 10 1826 1 METAL2;
  P_3 B 2 1826 1 METAL2;
  P_4 B 18 1826 1 METAL2;
  P_5 B 3037 1638 1 METAL2;
  P_6 B 3037 1319 1 METAL2;
  P_7 B 3037 1159 1 METAL2;
  P_8 B 3037 967 1 METAL2;
 ENDIOLIST;
ENDMODULE;
MODULE cc_13;
 TYPE GENERAL;
 DIMENSIONS 3037 0 3037 1826 -109 1826 -109 0;
 IOLIST;
  P_2 B 10 1826 1 METAL2;
  P_3 B 2 1826 1 METAL2;
  P_4 B 18 1826 1 METAL2;
  P_5 B 3037 1638 1 METAL2;
  P_6 B 3037 1319 1 METAL2;
  P_7 B 3037 1159 1 METAL2;
  P_8 B 3037 967 1 METAL2;
 ENDIOLIST;
ENDMODULE;
MODULE cc_14;
 TYPE GENERAL;
 DIMENSIONS 3037 0 3037 1826 -109 1826 -109 0;
 IOLIST;
  P_2 B 10 1826 1 METAL2;
  P_3 B 2 1826 1 METAL2;
  P_4 B 18 1826 1 METAL2;
  P_5 B 3037 1638 1 METAL2;
  P_6 B 3037 1319 1 METAL2;
  P_7 B 3037 1159 1 METAL2;
  P_8 B 3037 967 1 METAL2;
 ENDIOLIST;
ENDMODULE;
MODULE cc_21;
 TYPE GENERAL;
 DIMENSIONS 3077 0 3077 1832 -109 1832 -109 0;
 IOLIST;
  P_0 B 2 0 1 METAL2;
  P_1 B 42 0 1 METAL2;
  P_2 B 34 0 1 METAL2;
  P_3 B 58 0 1 METAL2;
  P_4 B 122 0 1 METAL2;
  P_5 B 58 1832 1 METAL2;
  P_6 B 34 1832 1 METAL2;
 ENDIOLIST;
ENDMODULE;
MODULE cc_22;
 TYPE GENERAL;
 DIMENSIONS 3077 0 3077 1832 -109 1832 -109 0;
 IOLIST;
  P_0 B 2 0 1 METAL2;
  P_1 B 42 0 1 METAL2;
  P_2 B 34 0 1 METAL2;
  P_3 B 58 0 1 METAL2;
  P_4 B 122 0 1 METAL2;
  P_5 B 58 1832 1 METAL2;
  P_6 B 34 1832 1 METAL2;
 ENDIOLIST;
ENDMODULE;
MODULE cc_23;
 TYPE GENERAL;
 DIMENSIONS 3077 0 3077 1832 -109 1832 -109 0;
 IOLIST;
  P_0 B 2 0 1 METAL2;
  P_1 B 42 0 1 METAL2;
  P_2 B 34 0 1 METAL2;
  P_3 B 58 0 1 METAL2;
  P_4 B 122 0 1 METAL2;
  P_5 B 58 1832 1 METAL2;
  P_6 B 34 1832 1 METAL2;
 ENDIOLIST;
ENDMODULE;
MODULE cc_24;
 TYPE GENERAL;
 DIMENSIONS 3077 0 3077 1832 -109 1832 -109 0;
 IOLIST;
  P_0 B 2 0 1 METAL2;
  P_1 B 42 0 1 METAL2;
  P_2 B 34 0 1 METAL2;
  P_3 B 58 0 1 METAL2;
  P_4 B 122 0 1 METAL2;
  P_5 B 58 1832 1 METAL2;
  P_6 B 34 1832 1 METAL2;
 ENDIOLIST;
ENDMODULE;
MODULE cc8;
 TYPE PARENT;
 DIMENSIONS 10000 -500 10000 10000 -500 10000 -500 -500;
 IOLIST;
  VSS PWR -500 8800 1 METAL2 CURRENT 40.000;
  VDD PWR 8400 -500 1 METAL2 CURRENT 509.000;
  GND PWR 10000 8800 1 METAL2 CURRENT 400.000;
 ENDIOLIST;
 NETWORK;
  c_0 cc_11 n0 n1 n2 n3 n4 n5 n6;
  c_1 cc_12 n0 n7 n8 n9 n10 n11 n12;
  c_2 cc_13 n1 n7 n13 n14 n15 n16 n17;
  c_3 cc_14 n2 n8 n13 n18 n19 n20 n21;
  c_4 cc_21 n3 n9 n14 n18 n22 n23 n24;
  c_5 cc_22 n4 n10 n15 n19 n22 n25 n26;
  c_6 cc_23 n5 n11 n16 n20 n23 n25 n27;
  c_7 cc_24 n6 n12 n17 n21 n24 n26 n27;
 ENDNETWORK;
ENDMODULE;
COMMUNICATIONS VOLUME;
n0 0 1 7
n1 0 2 1
n2 0 3 1
n3 0 4 6
n4 0 5 2
n5 0 6 1
n6 0 7 1
n7 1 2 7
n8 1 3 1
n9 1 4 5
n10 1 5 6
n11 1 6 2
n12 1 7 1
n13 2 3 7
n14 2 4 1
n15 2 5 5
n16 2 6 6
n17 2 7 2
n18 3 4 1
n19 3 5 1
n20 3 6 5
n21 3 7 5
n22 4 5 7
n23 4 6 2
n24 4 7 2
n25 5 6 7
n26 5 7 2
n27 6 7 7
END;