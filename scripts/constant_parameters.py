'''tested goal positions for some specific motion'''
Joint_Names = str({'left': ['left_s0', 'left_s1', 'left_e0', 'left_e1',
    	                     'left_w0', 'left_w1', 'left_w2'],
    	            'right': ['right_s0', 'right_s1', 'right_e0', 'right_e1',
    	                      'right_w0', 'right_w1', 'right_w2']})
joint_states = []

##state0
joint_states.append( [0.0, -0.08436894333369775, -1.491029325824622, 0.795369038518587, 0.24505343086469483, -0.04410194765170564, -0.10584467436409355, 1.1635244276110863, -0.03489806292439316, 1.2774225011115783, 1.4511458253396015, -0.6481068828815874, -0.21360682471304387, -1.3721458147635026, 1.3537380453088776, -1.514806028036846, -12.565987119160338]

	) 

##state1
joint_states.append([0.0, -0.0847524385306691, -1.4185487335970364, 0.8283496254581234, -0.1284708909854034, -0.07746602978821339, -0.15838351634916897, 1.3663933868089322, 0.0395000052880494, 1.2778059963085497, 1.4511458253396015, -0.6481068828815874, -0.21437381510698658, -1.3729128051574453, 1.3541215405058489, -1.514806028036846, -12.565987119160338]

) 

##state2
joint_states.append([0.0, 0.010737865515197896, -2.440563433525693, 2.042495419069428, 0.3777427690167831, -0.13268933815208828, 0.3271214030165645, 0.794218552927673, 1.7138400352649785, 1.2778059963085497, 1.4522963109305154, -0.6481068828815874, -0.21283983431910117, -1.3725293099604738, 1.3533545501119062, -1.5144225328398748, -12.565987119160338]
) 
##state3
joint_states.append([0.0, 0.009970875121255189, -2.306723609782691, 1.2106943368385628, -0.6481068828815874, 0.16643691548556738, -0.14150972768242942, 1.8584177245231788, 2.339704196722227, 1.2774225011115783, 1.4522963109305154, -0.6484903780785587, -0.2132233295160725, -1.3729128051574453, 1.3537380453088776, -1.5144225328398748, -12.565987119160338]

)

##state4
joint_states.append( [0.0, -0.08321845774278369, -1.9788352163721836, 1.5151895232338175, 0.08053399136398422, 0.4260631638351737, 0.5744758050630875, 1.1320778214594354, 1.2505778373235836, 1.9991604618116654, 1.32612639112694, -0.2554078011829214, 0.2607767339405203, -0.4855049193657335, 1.4373399982486328, -2.8858013572094348, -12.565987119160338]

)
##state5
joint_states.append([0.0, 0.009970875121255189, -2.037509981508801, 0.8889418665795973, 0.2024854640008746, 0.18407769454624964, 0.4421699621079705, 1.8005099497805044, 1.2559467700811826, 2.4635731453439744, 1.8691555900383767, -0.15378157398551273, -0.0559902987578176, -0.5181020111082985, 0.9334273094282742, -1.854199277356494, -12.565987119160338]

)
##state6
joint_states.append([0.0, 0.012655341500054663, 3.0150392385887805, 1.5416506918248407, -0.3160000423043952, 0.7900001057609881, 1.2114613272325054, 1.9914905578722384, 0.660762224381642, 2.5080585881926516, 0.9552865356556414, 0.8375535101854359, 0.2527233348041219, 0.03489806292439316, 2.0862138715241625, 0.8494418612915479, -12.565987119160338]

)

##state7
joint_states.append([0.0, -0.07286408742455715, 3.0146557433918093, 1.5401167110369554, -0.3160000423043952, 0.7911505913519021, 
					1.2114613272325054, 1.9926410434631525, 0.6603787291846707, -2.880432424451836, 1.8795099603566032, 0.7202039799122018, 
					0.9606554684132403, -1.4189322287940078, 1.8131652912805591, -2.2840973931613813, -12.565987119160338])

##state8
joint_states.append([0.0, -0.04716990922747647, 3.0269275896948926, 1.8971507394172855, 0.07363107781849985, 1.0469418877317949, 
					 1.4457768925820025, 1.8396264598715824, -0.7366942733819699, -2.5264663576472763, 1.3196069727784272, 1.4864273834609658, 
					 0.3903981105168378, -0.7282573790486002, 2.0655051308877095, -1.5769322499462053, -12.565987119160338])

##state9 this is left safe position!
joint_states.append([0.0, 0.04678641403050512, 3.056840215058658, 1.2233496783386175, -0.7021797056545481, 0.7102331047909466, 
					 0.7631554419729933, 1.3299613430966537, -2.0133497840996055, -2.664908123753935, 1.1297768502776073, 1.058063248443964, 
					 0.4314320965927726, -1.244441914172042, 1.3560390164907057, -0.9859661514133496, -12.565987119160338])

##state10 this is right safe position!
joint_states.append([0.0, -0.17640779060682257, 3.0526217678919734, 1.1757962739141696, -0.7980535048973865, 0.4812864721990486, 
					 0.8571117652309749, 1.1420486965806906, -2.0911993090847902, -2.5916605411324065, 1.1550875332777166, 0.9878836273982065, 
					 0.5572185211993765, -1.1850001586414822, 1.4856603930670231, -0.9936360553527768, -12.565987119160338])

##position0
loose_position = [0.0, 0.9936360553527768, 3.0541557486798587, 1.386718632248414, -0.9242234247009617, 0.8176117599429256,
					   0.8935438089432535, 1.1830826826566254, 1.204558413687021, 3.0591411862404865, 1.2501943421266122, 
					   0.5514660932448062, 0.8302671014429802, -0.6734175658816967, 1.4220001903697785, -1.3234419247481408, -12.565987119160338]
##position1
# loose_position.append([0.0, 0.9944030457467194, 3.049937301513174, 1.3928545553999556, -0.9154030351706206, 0.8110923415944126, 
# 					   0.8946942945341676, 1.1861506442323961, 1.1907525865960524, 3.0549227390738016, 0.29107285450125725, -0.04410194765170564, 
# 					   0.4425534573049419, -0.8302671014429802, 2.091966299478733, -1.3771312523241301, -12.565987119160338]
# )


right_hang = {'right_s0':loose_position[11], 'right_s1':loose_position[12], 'right_e0':loose_position[9], 'right_e1':loose_position[10],
                'right_w0':loose_position[13], 'right_w1':loose_position[14], 'right_w2':loose_position[15]}

left_hang = {'left_s0':loose_position[4], 'left_s1':loose_position[5], 'left_e0':loose_position[2], 'left_e1':loose_position[3],
                'left_w0':loose_position[6], 'left_w1':loose_position[7], 'left_w2':loose_position[8]}

left_safe_position = {'left_s0':joint_states[9][4], 'left_s1':joint_states[9][5], 'left_e0':joint_states[9][2], 'left_e1':joint_states[9][3],
                'left_w0':joint_states[9][6], 'left_w1':joint_states[9][7], 'left_w2':joint_states[9][8]}

right_safe_position = {'right_s0':joint_states[10][11], 'right_s1':joint_states[10][12], 'right_e0':joint_states[10][9], 'right_e1':joint_states[10][10],
                'right_w0':joint_states[10][13], 'right_w1':joint_states[10][14], 'right_w2':joint_states[10][15]}
                
right_prepare_position = {'right_s0':joint_states[9][11], 'right_s1':joint_states[9][12], 'right_e0':joint_states[9][9], 'right_e1':joint_states[9][10],
                'right_w0':joint_states[9][13], 'right_w1':joint_states[9][14], 'right_w2':joint_states[9][15]}
##### AR marker Information #####
cube_table = {'ar_marker_1':'red', 'ar_marker_2':'blue', 'ar_marker_3':'yellow', 
		      'ar_marker_4':'white', 'ar_marker_5':'orange',  'ar_marker_6':'green'}

##### Camera Information #####
camera_threads = {'left_hand_camera':None, 'right_hand_camera':None, 'head_camera':None}

#offsets
offsets = []
offsets.append([0,0,0.03])


########################################################################################### new strategy ################################################################################################################
new_joint_states=[]

####symmetric position about to change hands: left hand hold, right hand rotate red, 
#rotate right

new_joint_states.append( [0.0, -0.08321845774278369, -1.3349467806572812, 1.944320648644762, 0.22089323345549958, 0.1257864246066039, 3.0579907006495723, -0.7965195241095011, 1.9163254992658532, 1.0108933392164876, 2.0198692024481186, -0.603237944835939, -0.489339871335447, -2.5483255838746435, -1.1869176346263388, -0.8245146734884099, -12.565987119160338]
)



 # [0.0, -0.18906313210687725, -1.3337962950663673, 1.9431701630538478, 0.22050973825852824, 0.1250194342126612, 
	# 					  3.0579907006495723, -0.7961360289125298, 1.9167089944628244, 1.0408059645802532, 2.021403183236004, -0.6343010557906186, 
	# 					  -0.368922379486442, -2.607383844208232, -1.1244079175200083, -0.6481068828815874, -12.565987119160338])


#rotate bottom
new_joint_states.append( [0.0, 0.008436894333369775, -1.8123983008866162, 1.3709953291725885, 0.048703890015361885, 0.26422819071326253, 0.3681553890924993, 1.182699187459654, -0.15378157398551273, 1.4097283440666952, 1.355272026096763, -0.2795679985921167, -0.11236409271260656, 0.22549517581915582, 1.278189491505521, 1.3541215405058489, -12.565987119160338]
)


	# [0.0, 0.008436894333369775, -1.8342575271139834, 1.2455923997629559, 0.055223308363874894, 0.10469418877317949, 0.25080585881926515, 1.2455923997629559, -0.2166747862888147, 1.7602429540985123, 1.2221991927477034, -0.15416506918248407, 0.044485442848677, -0.1357572997278591, 1.2801069674903778, 1.6593837172950463, -12.565987119160338]





	# [0.0, -0.17640779060682257, -2.035592505523944, 1.3575729972785913, -0.6442719309118737, 0.9165535207615347, 
	# 					  2.536820727965503, -1.4730050515669686, -0.763922432366936, 1.780568199537994, 1.4979322393701064, -0.5568350260024052, 
	# 					  -0.11734953027323415, -3.0484033207252885, -1.328043867111797, -1.435422522263776, -12.565987119160338])

#rotate up 
new_joint_states.append([0.0, 0.008820389530341128, -1.5661943844310073, 1.309636097657172, -0.25617479157686407, -0.8406214717612067, 0.7113835903818606, 1.731864309522632, -2.492335285116826, 1.7755827619773663, 1.7291798431438326, -0.3846456825622675, 0.3631699515318717, -0.47169909227476475, 1.0941117969592713, 0.24965537322835107, -12.565987119160338]
)

 # [0.0, -0.17564080021287987, -1.9815196827509831, 1.3702283387786458, -0.48780589054756157, -0.4222282118654601, 
	# 					   1.2072428800658206, 1.9926410434631525, 0.3278883934105072, 1.4783739843245676, 1.7307138239317181, -0.6243301806693634, 
	# 					   -0.06020874592450249, -3.0487868159222598, -1.212995308020391, -1.818534224038158, -12.565987119160338])

####symmetric position about to change hands: right hand hold, left hand rotate white

#change hand
new_joint_states.append( [0.0, -0.08168447695489828, -1.7437526606287441, 1.2475098757478127, 0.14879613642488512, 0.008053399136398421, 0.13690778531877318, 1.4622671860517706, 1.384417661066586, 1.5497040909612392, 1.7966749978107908, -0.38119422578952533, -0.17909225698562206, 0.22357769983429907, 0.8176117599429256, 3.0526217678919734, -12.565987119160338]
)


	# [0.0, 0.008053399136398421, -1.7261118815680618, 1.3522040645209923, 0.2684466378799474, 
	# 					0.07861651537912745, 0.1614514779249398, 1.4055098969000104, 1.437723493445604, 1.5263108839459867, 
	# 					1.702335179355838, -0.33824276372873374, -0.06404369789421602, 0.12808739578843203, 0.952602069276842,
	#  					2.943709131952109, -12.565987119160338])

	# [0.0, -0.18829614171293454, -1.5761652595522626, 1.9025196721748845, 0.38502917775923884, 0.41724277430483253, 0.515417544729499, 
	# 					  1.013577805595287, -0.2274126518040126, 1.015495281580144, 1.9811361875540119, -0.5986360024722828, -0.34246121089541864, 
	# 					  -2.5920440363293777, -1.2007234617173077, 0.8732185635037718, -12.565987119160338])

#rotate down 
new_joint_states.append([0.0, 0.013805827090968724, -1.3993739737484687, 1.5857526394765464, 0.18714565612202047, 0.05177185159113271, 0.009203884727312482, 0.9276748814737039, -1.3805827090968723, 0.5031456984264157, 1.3560390164907057, 0.6051554208207958, -1.1478011245352608, -0.15684953556128356, 1.4423254358092603, -1.3322623142784817, -12.565987119160338]
)





	# [0.0, 0.008053399136398421, -1.572330307582549, 1.5585244804915803, 0.13230584295511694, 
	# 					0.2784175130012026, 0.39308257689563725, 0.9368787662010164, -0.07478156340941391, 0.5031456984264157, 
	# 					1.3560390164907057, 0.604388430426853, -1.1462671437473755, -0.15684953556128356, 1.4427089310062315, 
	# 					-1.3318788190815105, -12.565987119160338])


	# [0.0, -0.18752915131899184, -1.7268788719620045, 1.018946738352886, -0.017640779060682257, -0.0337475773334791, 
	# 					 0.06941263065181497, 1.6425099286283067, -0.09433981845495294, 1.6996507129770384, 1.492946801809479, 0.04678641403050512, 
	# 					 1.0469418877317949, 0.5307573526083531, 1.5270778743399294, -2.53605373757156, -12.565987119160338])

#rotate up 
new_joint_states.append([0.0, -0.18867963690990588, -1.4024419353242394, 0.9955535313376335, -0.17985924737956477, 0.17257283863710904, 
						 0.019941750242510378, 1.2030244328991357, 1.8396264598715824, 0.5483981316690354, 1.1481846197322323, 0.554534054820577, 
						 -0.9035146840645087, -0.32903887900142126, 1.4135632960364088, 1.94662161982659, -12.565987119160338])

#### Detect marker, right hand hold, left hand rotate
new_joint_states.append([0.0, -0.17564080021287987, -2.5172624729199637, 2.334335263964628, 0.10661166475803625, 0.794218552927673, 
						 2.004145899372293, 1.1339952974442922, -1.6164322552342547, 1.7245779007801765, 1.6570827461132183, -0.06902913545484361, 
						 -0.0038349519697135344, -3.0484033207252885, -0.1392087565006013, -0.13268933815208828, -12.565987119160338]) 
#### Detect marker, left hand hold, right hand rotate
new_joint_states.append(  [0.0, 0.008053399136398421, -1.604927399325114, 0.4801359866081345, -0.31254858553165304, -0.07439806821244256, 0.04295146206079158, 1.119805975156352, -0.05905826033358843, 1.558140985294609, 1.9742332740085275, -0.1928980840765908, -0.16528642989465334, -0.19750002644024703, -0.4126408319411763, -1.2593982268539248, -12.565987119160338]
)




 # [0.0, -0.18867963690990588, -1.5815341923098616, 1.5750147739613485, -0.06327670750027331, -0.21897575747064282, 
	# 					   -0.034514567727421806, 0.11083011192472114, -0.16451943950071063, 1.7709808196137102, 2.420621683283183, -0.8252816638823526, 
	# 					   0.12540292940963257, -2.4973207226774536, 0.15224759319762732, 0.9951700361406621, -12.565987119160338])

#### Safe Position, state8 ####
new_joint_states.append( [0.0, 0.008436894333369775, -2.1353012567364957, 0.7221214558970586, 0.29605829206188483, -0.11888351106111957, 0.6224127046845066, 1.6122138080675699, -0.4210777262745461, 0.7332428166092277, 1.1443496677625187, -0.6124418295632514, -0.3750583026379837, 0.4778350154263064, 1.5922720578250595, 2.5912770459354353, -12.565987119160338]
)


	# [0.0, -0.18867963690990588, -2.1372187327213528, 0.7198204847152304, 0.3037281960013119, -0.09088836168221076, 
	# 					  0.6224127046845066, 1.6586167269011036, -0.5365097805629234, 1.0223981951256282, 0.4536748180171111, -0.4678641403050512, 
	# 					  -0.02454369260616662, -0.3996019952441503, 1.641742938234364, -2.2833304027674384, -12.565987119160338])

#####right hand hold, to rotate, this is the same position as new_joint_states[3], only w2 is made to -1.25 in order to rotate in a linear space
new_joint_states.append( [0.0, 0.009970875121255189, -1.9109565665082542, 1.0695681043531047, -0.4563592843959106, -0.4743835586535642, 1.3008157081268308, 2.0931167850696473, 0.6289321230330196, 1.8841119027202595, 1.5274613695369008, -0.19903400722813244, 0.39883500485020756, -0.5069806503961293, 1.1953545289597087, -1.2931458041874038, -12.565987119160338]
)




	# [0.0, -0.18867963690990588, -1.3330293046724246, 2.0244711448117747, 0.2899223689103432, 0.06059224112147384, 3.0583741958465436, 
	# 					  -0.6914418401393503, -1.2528788085054117, 1.0151117863831725, 1.9811361875540119, -0.5986360024722828, -0.34207771569844725, 
	# 					  -2.5905100555414924, -1.2007234617173077, 0.8728350683068005, -12.565987119160338])









