%% appendix plots

close all; clear all;

% sim plot

int_pt=[16.60794607	18.46607258	20.42053924	21.47727606	22.08352457	22.47317647	22.86335007	23.09048622	23.31366093	23.49241818	23.58465263	23.68786357	23.79731203];
sqp=[16.60800124	18.46606503	20.42041638	21.47881635	22.08300787	22.47371399	22.8633596	23.09028837	23.31421103	23.49669262	23.58470265	23.68759899	23.79733483];
act_set=[16.60800124	18.46606503	20.42041638	21.47881635	22.08300787	22.47371399	22.8633596	23.09028837	23.31421103	23.49669262	23.58470265	23.68759899	23.79733483];
uncon=[16.60800124	18.46606503	20.42041638	21.47881635	22.08300787	22.47371399	22.8633596	23.09028837	23.31421103	23.49669262	23.58470265	23.68759899	23.79733483];
sim=[16.60800124	18.46606503	20.42041638	21.47881635	22.08300787	22.47371399	22.8633596	23.09028837	23.31421103	23.49669262	23.58470265	23.68759899	23.79733483];

figure (1)
hold on
plot(3:15,int_pt,"Color","#0072BD","LineStyle","--","Marker",".","MarkerSize",16)
plot(3:15,sqp,"Color","#D95319","LineStyle","--","Marker",".","MarkerSize",16)
plot(3:15,act_set,"Color","#EDB120","LineStyle","--","Marker",".","MarkerSize",16)
plot(3:15,uncon,"Color","#7E2F8E","LineStyle","--","Marker",".","MarkerSize",16)
plot(3:15,sim,"Color","#A2142F","LineStyle","--","Marker",".","MarkerSize",16)
title('Mission Time Improvement over a Straight-Line')
xlabel('Number of Design Points, N')
ylabel('Mission Time Improvement, [sec]')
grid minor
legend('Interior Points','SQP','Active Set','Unconstrained','Simulated Annealing')

% timing plot

int_pt_t=[28.311358	36.725471	66.487560	90.231189	122.793871	157.026711	189.660526	257.678926	285.881492	336.567927	442.932759	456.282213	465.804714];
sqp_t=[44.233866	58.591315	102.842705	158.305249	168.744947	192.202256	210.035910	244.848983	309.164189	358.673856	383.636872	429.987040	419.997019];
act_set_t=[31.973024	47.429183	75.735972	90.673246	98.144069	117.880623	124.880616	162.630051	192.586147	242.586147	247.594479	289.468235	309.736351];
uncon_t=[25.911993	36.843662	38.629089	28.588642	34.046975	43.242640	57.483100	71.189533	81.564717	99.269398	94.489067	108.292472	137.326941];
sim_t=[380.7182006	414.2819182	688.2132151	707.7814508	877.9818736	966.2098983	994.3342121	949.8142636 ];

figure (2)
hold on
plot(3:15,int_pt_t,"Color","#0072BD","LineStyle","--","Marker",".","MarkerSize",16)
plot(3:15,sqp_t,"Color","#D95319","LineStyle","--","Marker",".","MarkerSize",16)
plot(3:15,act_set_t,"Color","#EDB120","LineStyle","--","Marker",".","MarkerSize",16)
plot(3:15,uncon_t,"Color","#7E2F8E","LineStyle","--","Marker",".","MarkerSize",16)
plot(3:10,sim_t,"Color","#A2142F","LineStyle","--","Marker",".","MarkerSize",16)
title('Computational Cost of Descritizing the Design Path')
xlabel('Number of Design Points, N')
ylabel('Computation Time, [sec]'); ylim([0, 600])
grid minor
legend('Interior Points','SQP','Active Set','Unconstrained','Simulated Annealing')

