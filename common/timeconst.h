//*************************************************************************************************
//* timeconst.h                                                                                   *
//*                                                                                               *
//* tabulky pro prevod casu z formatu RTC na pocet sekund od roku 2000                            *
//*                                                                                               *
//*************************************************************************************************
//* History:                                                                                      *
//* 07.05.2024  JD  portovana verze z CC3220                                                      *
//*                                                                                               *
//*************************************************************************************************


#include "pico/stdlib.h"

// pocet sekund v danem roce
const uint32_t rok_sec[100] = {

  0x00000000,                //  0            2000 rok
  0x01E28500,                //  31622400     2001
  0x03C3B880,                //  63158400     2002
  0x05A4EC00,                //  94694400     2003
  0x07861F80,                //  126230400    2004
  0x0968A480,                //  157852800    2005
  0x0B49D800,                //  189388800    2006
  0x0D2B0B80,                //  220924800    2007
  0x0F0C3F00,                //  252460800    2008
  0x10EEC400,                //  284083200    2009
  0x12CFF780,                //  315619200    2010
  0x14B12B00,                //  347155200    2011
  0x16925E80,                //  378691200    2012
  0x1874E380,                //  410313600    2013
  0x1A561700,                //  441849600    2014
  0x1C374A80,                //  473385600    2015
  0x1E187E00,                //  504921600    2016
  0x1FFB0300,                //  536544000    2017
  0x21DC3680,                //  568080000    2018
  0x23BD6A00,                //  599616000    2019
  0x259E9D80,                //  631152000    2020
  0x27812280,                //  662774400    2021
  0x29625600,                //  694310400    2022
  0x2B438980,                //  725846400    2023
  0x2D24BD00,                //  757382400    2024
  0x2F074200,                //  789004800    2025
  0x30E87580,                //  820540800    2026
  0x32C9A900,                //  852076800    2027
  0x34AADC80,                //  883612800    2028
  0x368D6180,                //  915235200    2029
  0x386E9500,                //  946771200    2030
  0x3A4FC880,                //  978307200    2031
  0x3C30FC00,                //  1009843200   2032
  0x3E138100,                //  1041465600   2033
  0x3FF4B480,                //  1073001600   2034
  0x41D5E800,                //  1104537600   2035
  0x43B71B80,                //  1136073600   2036
  0x4599A080,                //  1167696000   2037
  0x477AD400,                //  1199232000   2038
  0x495C0780,                //  1230768000   2039
  0x4B3D3B00,                //  1262304000   2040
  0x4D1FC000,                //  1293926400   2041
  0x4F00F380,                //  1325462400   2042
  0x50E22700,                //  1356998400   2043
  0x52C35A80,                //  1388534400   2044
  0x54A5DF80,                //  1420156800   2045
  0x56871300,                //  1451692800   2046
  0x58684680,                //  1483228800   2047
  0x5A497A00,                //  1514764800   2048
  0x5C2BFF00,                //  1546387200   2049
  0x5E0D3280,                //  1577923200   2050
  0x5FEE6600,                //  1609459200   2051
  0x61CF9980,                //  1640995200   2052
  0x63B21E80,                //  1672617600   2053
  0x65935200,                //  1704153600   2054
  0x67748580,                //  1735689600   2055
  0x6955B900,                //  1767225600   2056
  0x6B383E00,                //  1798848000   2057
  0x6D197180,                //  1830384000   2058
  0x6EFAA500,                //  1861920000   2059
  0x70DBD880,                //  1893456000   2060
  0x72BE5D80,                //  1925078400   2061
  0x749F9100,                //  1956614400   2062
  0x7680C480,                //  1988150400   2063
  0x7861F800,                //  2019686400   2064
  0x7A447D00,                //  2051308800   2065
  0x7C25B080,                //  2082844800   2066
  0x7E06E400,                //  2114380800   2067
  0x7FE81780,                //  2145916800   2068
  0x81CA9C80,                // -2117428096   2069
  0x83ABD000,                // -2085892096   2070
  0x858D0380,                // -2054356096   2071
  0x876E3700,                // -2022820096   2072
  0x8950BC00,                // -1991197696   2073
  0x8B31EF80,                // -1959661696   2074
  0x8D132300,                // -1928125696   2075
  0x8EF45680,                // -1896589696   2076
  0x90D6DB80,                // -1864967296   2077
  0x92B80F00,                // -1833431296   2078
  0x94994280,                // -1801895296   2079
  0x967A7600,                // -1770359296   2080
  0x985CFB00,                // -1738736896   2081
  0x9A3E2E80,                // -1707200896   2082
  0x9C1F6200,                // -1675664896   2083
  0x9E009580,                // -1644128896   2084
  0x9FE31A80,                // -1612506496   2085
  0xA1C44E00,                // -1580970496   2086
  0xA3A58180,                // -1549434496   2087
  0xA586B500,                // -1517898496   2088
  0xA7693A00,                // -1486276096   2089
  0xA94A6D80,                // -1454740096   2090
  0xAB2BA100,                // -1423204096   2091
  0xAD0CD480,                // -1391668096   2092
  0xAEEF5980,                // -1360045696   2093
  0xB0D08D00,                // -1328509696   2094
  0xB2B1C080,                // -1296973696   2095
  0xB492F400,                // -1265437696   2096
  0xB6757900,                // -1233815296   2097
  0xB856AC80,                // -1202279296   2098
  0xBA37E000                 // -1170743296   2099

};


// pocet sekund v danem mesici (pro neprestupny rok)
const uint32_t mesic_sec[13] = {

  0x00000000,                //  0            (nesmyslny mesic)
  0x00000000,                //  0            1 mesic
  0x0028DE80,                //  2678400      2
  0x004DC880,                //  5097600      3
  0x0076A700,                //  7776000      4
  0x009E3400,                //  10368000     5
  0x00C71280,                //  13046400     6
  0x00EE9F80,                //  15638400     7
  0x01177E00,                //  18316800     8
  0x01405C80,                //  20995200     9
  0x0167E980,                //  23587200     10
  0x0190C800,                //  26265600     11
  0x01B85500                 //  28857600     12

};


// pocet sekund v danem dnu
const uint32_t den_sec[32] = {

  0x00000000,                //  0            (nesmyslny den)
  0x00000000,                //  0            1 den
  0x00015180,                //  86400        2
  0x0002A300,                //  172800       3
  0x0003F480,                //  259200       4
  0x00054600,                //  345600       5
  0x00069780,                //  432000       6
  0x0007E900,                //  518400       7
  0x00093A80,                //  604800       8
  0x000A8C00,                //  691200       9
  0x000BDD80,                //  777600       10
  0x000D2F00,                //  864000       11
  0x000E8080,                //  950400       12
  0x000FD200,                //  1036800      13
  0x00112380,                //  1123200      14
  0x00127500,                //  1209600      15
  0x0013C680,                //  1296000      16
  0x00151800,                //  1382400      17
  0x00166980,                //  1468800      18
  0x0017BB00,                //  1555200      19
  0x00190C80,                //  1641600      20
  0x001A5E00,                //  1728000      21
  0x001BAF80,                //  1814400      22
  0x001D0100,                //  1900800      23
  0x001E5280,                //  1987200      24
  0x001FA400,                //  2073600      25
  0x0020F580,                //  2160000      26
  0x00224700,                //  2246400      27
  0x00239880,                //  2332800      28
  0x0024EA00,                //  2419200      29
  0x00263B80,                //  2505600      30
  0x00278D00                 //  2592000      31

};


// pocet sekund v dane hodine
const uint32_t hod_sec[24] = {

  0x00000000,                //  0            0 hod
  0x00000E10,                //  3600         1
  0x00001C20,                //  7200         2
  0x00002A30,                //  10800        3
  0x00003840,                //  14400        4
  0x00004650,                //  18000        5
  0x00005460,                //  21600        6
  0x00006270,                //  25200        7
  0x00007080,                //  28800        8
  0x00007E90,                //  32400        9
  0x00008CA0,                //  36000        10
  0x00009AB0,                //  39600        11
  0x0000A8C0,                //  43200        12
  0x0000B6D0,                //  46800        13
  0x0000C4E0,                //  50400        14
  0x0000D2F0,                //  54000        15
  0x0000E100,                //  57600        16
  0x0000EF10,                //  61200        17
  0x0000FD20,                //  64800        18
  0x00010B30,                //  68400        19
  0x00011940,                //  72000        20
  0x00012750,                //  75600        21
  0x00013560,                //  79200        22
  0x00014370                 //  82800        23

};


// pocet sekund v dane minute
const uint32_t min_sec[61] = {

  0x0000,                    //  0            0 min
  0x003C,                    //  60           1
  0x0078,                    //  120          2
  0x00B4,                    //  180          3
  0x00F0,                    //  240          4
  0x012C,                    //  300          5
  0x0168,                    //  360          6
  0x01A4,                    //  420          7
  0x01E0,                    //  480          8
  0x021C,                    //  540          9
  0x0258,                    //  600          10
  0x0294,                    //  660          11
  0x02D0,                    //  720          12
  0x030C,                    //  780          13
  0x0348,                    //  840          14
  0x0384,                    //  900          15
  0x03C0,                    //  960          16
  0x03FC,                    //  1020         17
  0x0438,                    //  1080         18
  0x0474,                    //  1140         19
  0x04B0,                    //  1200         20
  0x04EC,                    //  1260         21
  0x0528,                    //  1320         22
  0x0564,                    //  1380         23
  0x05A0,                    //  1440         24
  0x05DC,                    //  1500         25
  0x0618,                    //  1560         26
  0x0654,                    //  1620         27
  0x0690,                    //  1680         28
  0x06CC,                    //  1740         29
  0x0708,                    //  1800         30
  0x0744,                    //  1860         31
  0x0780,                    //  1920         32
  0x07BC,                    //  1980         33
  0x07F8,                    //  2040         34
  0x0834,                    //  2100         35
  0x0870,                    //  2160         36
  0x08AC,                    //  2220         37
  0x08E8,                    //  2280         38
  0x0924,                    //  2340         39
  0x0960,                    //  2400         40
  0x099C,                    //  2460         41
  0x09D8,                    //  2520         42
  0x0A14,                    //  2580         43
  0x0A50,                    //  2640         44
  0x0A8C,                    //  2700         45
  0x0AC8,                    //  2760         46
  0x0B04,                    //  2820         47
  0x0B40,                    //  2880         48
  0x0B7C,                    //  2940         49
  0x0BB8,                    //  3000         50
  0x0BF4,                    //  3060         51
  0x0C30,                    //  3120         52
  0x0C6C,                    //  3180         53
  0x0CA8,                    //  3240         54
  0x0CE4,                    //  3300         55
  0x0D20,                    //  3360         56
  0x0D5C,                    //  3420         57
  0x0D98,                    //  3480         58
  0x0DD4,                    //  3540         59
  0x0E10                     //  3600         60

};

