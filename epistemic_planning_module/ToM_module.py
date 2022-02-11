import os
import subprocess

# from std_msgs.msg import String

# import rospy
# from gazebo_event_controller.srv import EventServer, EventServerRequest, EventServerResponse
# from gazebo_event_controller.srv import ActionServer, ActionServerRequest

global_current_domain = "bolander_spoon_cabinet" #"corridor"

# 
initial_state_bolander = "(at a p1) (at b p1) [a](at a p1) [a](at b p1) [b](at a p1) [b](at b p1) (atBox bx1 p1) (atBox bx2 p1) (atBox bx3 p1) [a](atBox bx1 p1) [a](atBox bx2 p1) [a](atBox bx3 p1) [b](atBox bx1 p1) [b](atBox bx2 p1) [b](atBox bx3 p1) (in b1 bx1) (in b2 bx2) (in b3 bx3) [a](in b1 bx1) [a](in b2 bx2) [a](in b3 bx3) [b](in b1 bx1) [b](in b2 bx2) [b](in b3 bx3) [a](!holding a b1) [b](!holding a b1) (dummy) (atRobot p1)"

#initial_state_bolander_spoon_cabinet = "(at a p1) (at b p1) [a](at a p1) [a](at b p1) [b](at a p1) [b](at b p1) (atCabinet cabinet1 p1) (atCabinet cabinet2 p1) [a](atCabinet cabinet1 p1) [a](atCabinet cabinet2 p1) [b](atCabinet cabinet1 p1) [b](atCabinet cabinet2 p1)  (holding b spoon1)  [a](holding b spoon1) [b](holding b spoon1) (dummy) (atRobot p1) (in bowl1 cabinet3) [a](in bowl1 cabinet3) [b](in bowl1 cabinet3)"
initial_state_bolander_spoon_cabinet = "(at a p1) (at b p1) [a](at a p1) [a](at b p1) [b](at a p1) [b](at b p1) (atCabinet cabinet1 p1) (atCabinet cabinet2 p1) [a](atCabinet cabinet1 p1) [a](atCabinet cabinet2 p1) [b](atCabinet cabinet1 p1) [b](atCabinet cabinet2 p1)  (holding b bowl1)  [a](holding b bowl1) [b](holding b bowl1) (dummy) (atRobot p1) (in spoon1 cabinet3) [a](in spoon1 cabinet3) [b](in spoon1 cabinet3)"

#initial_state_charger_scenario = "(atRobotQuadrant q1) (at a p1) [a](at a p1) (atRobot p3) (chargerInQuadrant q5) chargerInQuadrant q1) (not (robotHoldingCharger))  (objInQuadrant charger1 q1) [a](!objInQuadrant charger1 q1) (facing a q2) (dummy) (atRobot p1) (not (khowPreservingGoal))"
initial_state_charger = "(atRobotQuadrant q1) (chargerInQuadrant q5) (chargerInQuadrant q1) (!robotHoldingCharger)   (facing a q2) (dummy) (atRobot p1)"#" (!khowPreservingGoal) (!disc_resolution) (!disc_resolution_two)"

initial_state_corridor_scenario = "(dummy) (door_open) [a](door_open) (at a q1) (at b q1) [a](at b q1) [a](at c q1) [a](!at c q5) [b](!at c q5) (at c q1) (disc_resolution_two)"
initial_state_kitchen = "(and (dummy) (not (pepper_near_cabinet)) (not (elim_t2_t3_t4)) (not (elim_t1_t3_t4)) (not (elim_t1_t2_t4)) (not (elim_t1_t2_t3)) (K_pepper (or (and (at_bowl_cabinet) (at_spoon_cabinet) (K_human (and (at_bowl_cabinet) (at_spoon_cabinet) (not (at_spoon_dishwasher)) (not (at_bowl_dishwasher)) ) )) (and (at_bowl_cabinet) (not (at_spoon_cabinet)) (at_spoon_dishwasher) (K_human (and (at_bowl_cabinet) (not (at_spoon_cabinet)) (at_spoon_dishwasher) ) )) (and (not (at_bowl_cabinet)) (at_spoon_cabinet) (at_bowl_bedroom) (K_human (and (not (at_bowl_cabinet)) (at_spoon_cabinet) (at_bowl_dishwasher) ) )) (and (not (at_bowl_cabinet)) (not (at_spoon_cabinet)) (at_bowl_bedroom) (at_spoon_dishwasher) (K_human (and (not (at_bowl_cabinet)) (not (at_spoon_cabinet)) (at_bowl_dishwasher) (at_spoon_dishwasher) ) )) ) ) )"

ROS = False
live = True
simplified = True
newEmpatheticPlan = False

def send_action_request(object,agent,action,final_loc):
	action_req = ActionServerRequest()
	scenario_action_service = rospy.ServiceProxy('/action_server', ActionServer)
	action_req.object = object #"charger"
	action_req.agent = agent # "robot"
	action_req.action = action #"pickup"
	action_req.endgoal = final_loc #""
	reponse = scenario_action_service(action_req)

	return response

def run_planner(agent,goal,template_file,plan_to_execute,domain_file_name,validation):

	curr_state_file = open('curr_state.txt','r')
	raw_lines = curr_state_file.readlines()
	lines = [line.strip() for line in raw_lines]
	curr_state = lines[0]

	if validation:
		problem_file = open("{0}/curr_problem.pdkbddl".format(global_current_domain), "w")
		template_file = open("{0}/prob_template.pdkbddl".format(global_current_domain), "r")
	else:
		problem_file = open('{0}/curr_problem_generation.pdkbddl'.format(global_current_domain),'w')
		template_file = open("{0}/prob_template_generation.pdkbddl".format(global_current_domain), "r")
		

	raw_lines = template_file.readlines()
	lines = [line.strip() for line in raw_lines]

	for line in lines:
		if agent == -1:
			if validation:
				problem_file.write(line.replace("<TEMPLATE>","(:plan " + plan_to_execute + " )").replace("<INIT_TEMPLATE>",curr_state).replace("<GOAL_TEMPLATE>","{0}".format(goal)).replace("include:domain.pdkbddl","include:{0}".format(domain_file_name)))
				problem_file.write('\n')
			else:
				# print(line.replace("<TEMPLATE>","").replace("<INIT_TEMPLATE>",curr_state).replace("<GOAL_TEMPLATE>","{0}".format(goal).replace("include:domain.pdkbddl","include:{0}".format(domain_file_name))))
				problem_file.write(line.replace("<TEMPLATE>","").replace("<INIT_TEMPLATE>",curr_state).replace("<GOAL_TEMPLATE>","{0}".format(goal)).replace("include:domain.pdkbddl","include:{0}".format(domain_file_name)))
				problem_file.write('\n')
		else:
			if validation:
				problem_file.write(line.replace("<TEMPLATE>","(:plan " + plan_to_execute + " )").replace("<INIT_TEMPLATE>",curr_state).replace("<GOAL_TEMPLATE>","[{0}]{1}".format(agent,goal).replace("include:domain.pdkbddl","include:{0}".format(domain_file_name))))
				problem_file.write('\n')
			else:
				problem_file.write(line.replace("<TEMPLATE>","").replace("<INIT_TEMPLATE>",curr_state).replace("<GOAL_TEMPLATE>","[{0}]{1}".format(agent,goal)).replace("include:domain.pdkbddl","include:{0}".format(domain_file_name)))
				problem_file.write('\n')

	problem_file.close()

	if validation:
		os.system("python ../pdkb-planning/pdkb/planner.py {0}/curr_problem.pdkbddl --keep-files > final_state.txt".format(global_current_domain))
	else:
		os.system("python ../pdkb-planning/pdkb/planner.py {0}/curr_problem_generation.pdkbddl --keep-files > predicted_plan.txt".format(global_current_domain))



def is_plan_valid(plan_to_execute,goal):
	
	run_planner(-1,goal,"prob_template.pdkbddl",plan_to_execute,"domain_prediction_lifted.pdkbddl",True)
	

	f = open("final_state.txt")

	raw_lines = f.readlines()
	lines = [line.strip() for line in raw_lines]

	for line in lines:
		if line.find("Goal holds") != -1:
			if line.find("True") != -1:
				return True
			else:
				return False


def parse_plan_mepk():
	f = open("predicted_plan.txt")

	raw_lines = f.readlines()
	lines = [line.strip() for line in raw_lines]


	plan_start = False

	predicted_plan = []

	# we can use this solution to create a tree structure in python from the indented text file created by MEPK
	# https://codereview.stackexchange.com/questions/176391/parsing-indented-text-and-inserting-its-data-into-a-tree

	for line in lines:
		if line.find("Statistic") != -1:
			break
		if plan_start:
			if len(line) > 0:
				
				predicted_plan.append(line)



		if line.find("Plan tree:") != -1:
			plan_start = True

	print(predicted_plan)

	return predicted_plan

def parse_plan():
	f = open("predicted_plan.txt")

	raw_lines = f.readlines()
	lines = [line.strip() for line in raw_lines]


	plan_start = False

	predicted_plan = ""

	for line in lines:
		if plan_start:
			if len(line) > 0:
				idx = line.find("(")
				predicted_plan += (line[idx:] + " ")



		if line.find("--{ Plan }--") != -1:
			plan_start = True

	return predicted_plan




def parse_final_state():
	f = open("final_state.txt")

	raw_lines = f.readlines()
	lines = [line.strip() for line in raw_lines]


	plan_start = False

	final_state = ""

	for line in lines:
		if plan_start:
			if len(line) > 0:
				if line[0] == 'B':
					tmp_str = '[{0}]'.format(line[1]) + '(' + line[3:].replace('_',' ') + ')'
					final_state += (" " + tmp_str)
					# print(tmp_str)

				else:
					if line[0] == 'P':
						continue 
					final_state += (" " + '(' + line.replace('_',' ') + ')')
					# print('(' + line.replace('_',' ') + ')')



		if line.find("Final state") != -1:
			plan_start = True

	return final_state



def update_problem_file(observed_actions):#,domain_file,problem_file):
	
	if observed_actions[0].strip().lower() == '(takeobjectoutofcabinet_b_spoon1_cabinet3_p1)'.strip().lower(): #observed_actions[0].find('takeobjectoutofcabinet') !=1 and observed_actions[0].find("spoon") != -1:
		return 0


	if live:

		if global_current_domain != "kitchen":
			plan_to_execute = ""
			for act in observed_actions:
				plan_to_execute += (act.lower() + " ")


			run_planner(-1,"(dummy)","prob_template.pdkbddl",plan_to_execute,"domain_for_state_update.pdkbddl",validation=True)
			

			next_state = parse_final_state()


			curr_state = next_state

			curr_state_file = open('curr_state.txt','w')
			curr_state_file.write(curr_state)

			curr_state_file.close()

			

		else:
			cmd = "printf 'e1+\ne3+' | ./test kitchen_update_state.epddl > output.txt | pkill -f './test kitchen_update_state.epddl'"
			timeoutSeconds = 1
			try:
				subprocess.check_output(cmd, shell=True, timeout=timeoutSeconds)
				# time.sleep(0.5)
				# os.system("pkill -f './test kitchen_update_state.epddl'")
			except:
				a=0

			mepk_state_output_file = open("output.txt", "r")
			raw_lines = mepk_state_output_file.readlines()
			lines = [line.strip() for line in raw_lines]

			relevant_idx1 = lines.index('>> KB entails precondition of action "check_cabinet_bowl(human)" :') 
			relevant_idx = lines[lines.index('>> KB entails precondition of action "check_cabinet_bowl(human)" :'):].index('∇_pepper :')
			relevant_lines = lines[relevant_idx+relevant_idx1:]
			rrelevant_lines = relevant_lines[0:relevant_lines.index('----------------------------------------------------------------------------')]
			rrrelevant_lines = [line for line in rrelevant_lines if line.find("ACDF") == -1]

			

			state = "(K_pepper (or "

			indices = [i for i, x in enumerate(rrrelevant_lines) if x == '∇_human :']

			for index in indices:
				state += " (and "
				temp_str = rrrelevant_lines[index-1][1:-1].replace("&", "")
				temp_str_list = [conjunct for conjunct in rrrelevant_lines[index-1][1:-1].replace("&", "").split(" ") if len(conjunct) > 0]
				
				# temp_indices = [i for i, x in enumerate(temp_str) if x == '~']
				# for idx in temp_indices:
				# 	almost_final_str = temp_str[idx].
				# 	idx_of_closing_bracket = find(")")

				final_str = ""
				for conjunct in temp_str_list:
					if conjunct.find("~") != -1:
						final_str += " " + conjunct.replace("~","(not") + ") "
					else:
						final_str += " " + conjunct + " "

				state += " {0} ".format(final_str)
 
				state += " (K_human (and "
				temp_str = rrrelevant_lines[index+1][1:-1].replace("&", "")
				temp_str_list = [conjunct for conjunct in rrrelevant_lines[index+1][1:-1].replace("&", "").split(" ") if len(conjunct) > 0]

				final_str = ""
				for conjunct in temp_str_list:
					if conjunct.find("~") != -1:
						final_str += " " + conjunct.replace("~","(not") + ") "
					else:
						final_str += " " + conjunct + " "

				state += " {0} ".format(final_str)
				state += ") "
				state += ") "
				state += ") "


			state += "))"

			curr_state_file = open('curr_state.txt','w')
			curr_state_file.write(state)

			curr_state_file.close()

		print("===========================================\n")
		os.system("cat curr_state.txt")
		print("\n===========================================\n")




def predict_agent_plan(agent,goal):
	# replace goal formula in template file with [agent]goal
	# replace init state in template file with the state from curr_state.txt
	run_planner(agent,goal,"prob_template_generation.pdkbddl",[],"domain_prediction_lifted.pdkbddl",False)
	return parse_plan()
	

def is_plan_ill_formed(predicted_plan,goal):
	#run rp-mep in valid_assess mode and check whether goal is achieved (Goal holds: False or Goal holds: True)

	#MAYBE PASS TO THIS THE CORRECT DOMAIN FILE and definitely with correct problem file where the goal is without [agent] 
	return not(is_plan_valid(predicted_plan,goal)) # MAYBE ALSO PASS agent so we can check if pepper believes that mary believes that the plan is valid 

	#return True if Goal holds: False

def gen_empathetic_plan(goal):
	run_planner(-1,goal,"prob_template_generation.pdkbddl",[],"domain_prediction_lifted.pdkbddl",False)
	return parse_plan()

def encode_domain(plan):#,domain_template_file):
	
	domain_template_file = open("domain_lifted_template.pdkbddl", "r")
	encoded_domain_file = open("encoded_domain.pdkbddl", "w")

		
	raw_lines = domain_template_file.readlines()
	lines = [line.strip() for line in raw_lines]

	actions = plan.split(" ")

	new_lines = []

	special_prop_cnt = 0

	for act in actions:
		act_name = act.split("_")[0].replace("(","").replace(")","")
		act_params = [param.replace("(","").replace(")","") for param in act.split("_")[1:]]
		#print(act_params)

		# print(act_name)
		# print(act_params)

		action_start = False

		for line in lines:
			new_line = line	
			
		
			if line.find("(:action") != -1:
				action_start = False

			if action_start:
				if line.find(":parameters") != -1:
					idx = line.find("(")
					line_param = line[idx:]
					params = line_param.split(" ")
					params_filtered = [param.replace("(","").replace(")","") for param in params if param.find("?") != -1]

				if line.find(":parameters") == -1 and line.find(":derive-condition") == -1:
					for param in params_filtered:
						if new_line.find(param) != -1:
							new_line = new_line.replace(param,act_params[params_filtered.index(param)])
							# print(new_line)
							# print(act_params[params_filtered.index(param)])

				if line.find(":effect") != -1:
					# print(line)
					new_line = line.replace("(and", "(and (special_prop_{0}) ".format(special_prop_cnt))


			
			
			if line.find("(:action") != -1 and line.find(act_name) != -1:
				#print(line)
				action_start = True
				special_prop_cnt += 1

			new_lines.append(new_line)
		

	special_prop_string = ""
	for i in range(special_prop_cnt):
		special_prop_string += "(special_prop_{0}) ".format(i+1)

	for line in new_lines:
		new_line = line
		if line.find("(:predicates") != -1:
			new_line = line.replace("(:predicates","(:predicates {0}".format(special_prop_string))
		encoded_domain_file.write(new_line)
		encoded_domain_file.write("\n")

	

	encoded_domain_file.close()

	return special_prop_string
				



def resolve_disc(agent,emp_plan,goal):#,predicted_plan):
	#we might want to have a function that automatically takens plan and other arguments and encodes the template domain as we describe in the paper
	#then we call RP-mEP and get the disc resolving plan.
	#run_planner()

	special_prop_string = encode_domain(emp_plan.strip())#,"domain_prediction_lifted.pdkbddl")
	new_goal = goal + " {0}".format(special_prop_string)
	#print(new_goal)
	run_planner(agent,new_goal,"prob_template_generation.pdkbddl",[],"encoded_domain.pdkbddl",False)
	
def parse_inform_action(act):
	explanation = ""
	if act.find("blocklocation") != -1:
		inform_content = act.replace("(informblocklocation b","")
		inform_content_split = inform_content.strip().split(" ")

		# DEPENDING ON THE REQ.SCENARIO, we say either Box/cabinet/
		explanation = "Block {0} is in Box {1}".format(inform_content_split[0].replace("(",""),inform_content_split[1].replace(")",""))
		

	if act.find("blocknotinlocation") != -1:
		
		inform_content = act.replace("(informblocknotinlocation b","")
		
		inform_content_split = inform_content.strip().split(" ")

		# DEPENDING ON THE REQ.SCENARIO, we say either Box/cabinet/
		explanation = "Block {0} is NOT in Box {1}".format(inform_content_split[0].replace("(",""),inform_content_split[1].replace(")",""))
		
		
		

	if act.find("bowllocation") != -1:
		inform_content = act.replace("(informbowllocation b","")
		inform_content_split = inform_content.strip().split(" ")

		explanation = "If you are looking for the bowl, it is in {0}".format(inform_content_split[1].replace("(",""))
		
		
		

	if act.find("bowlnotinlocation") != -1:
		
		inform_content = act.replace("(informbowlnotinlocation b","")
		
		inform_content_split = inform_content.strip().split(" ")

		explanation = "If you are looking for the bowl, it is NOT in {0}".format(inform_content_split[1].replace("(",""))
		
		

	if act.find("chargerinquadrant") != -1:
		inform_content = act.replace("(informchargerinquadrant a","")
		inform_content_split = inform_content.strip().split(" ")

		if inform_content_split[0].find("q1") != -1:
			charger_loc = "living room"
		if inform_content_split[0].find("q5") != -1:
			charger_loc = "bedroom"

		explanation = "If you need to charge your phone, know that there is a phone charger in {0}".format(charger_loc)
		

	if act.find("chargernotinquadrant") != -1:
		
		inform_content = act.replace("(informchargernotinquadrant a","")
		
		inform_content_split = inform_content.strip().split(" ")

		if inform_content_split[0].find("q1") != -1:
			charger_loc = "living room"
		if inform_content_split[0].find("q5") != -1:
			charger_loc = "bedroom"

		explanation = "If you need to charge your phone, know that there is NO phone charger in {0}".format(charger_loc)

	if act.find("agentloc") != -1:
		inform_content = act.replace("(informagentloc a","")
		inform_content_split = inform_content.strip().split(" ")

		explanation = "Eve is right outside the door!"
		


	print(explanation)
	return explanation
		
		


def send_pepper_plan_to_action_server(plan,goal):
	print(plan)
	actions = plan.split(")")

	for act in actions:
		if act.find('special') != -1 or act.find('}') != -1:
			continue
		if act.find("inform") != -1:
			parse_inform_action(act)
			# send_action_request(parse_inform_action(act),speech/communication)
			continue
		
		print(act.strip().replace('(','').split())

	# # TODO send actions as action requests
	# send_action_request("charger","robot","dropOff","livingroom")
	# send_action_request("The charger is not in livingroom, it is in the bedroom",speech/communication)



def disc_resolving_plan_to_natural_language(plan,goal):
	# plan = parse_plan() # we might need to do this for the old version
	actions = plan.split(")")



	#natural_language_utter = "The plan {0} will fail to achieve goal {1} since"
	#natural_language_utter = "If your goal is {0}, then you better do {1} because <EXPLANATION>".format(goal,emp_plan)
	natural_language_utter = "<EXPLANATION>"

	for act in actions:
		if act.find("discresolutionspecial") != -1:
			continue
		if act.find("inform") != -1:
			if act.find("blocklocation") != -1:
				inform_content = act.replace("(informblocklocation b","")
				inform_content_split = inform_content.strip().split(" ")

				# DEPENDING ON THE REQ.SCENARIO, we say either Box/cabinet/
				explanation = "Block {0} is in Box {1}".format(inform_content_split[0].replace("(",""),inform_content_split[1].replace(")",""))
				print(explanation)

			if act.find("blocknotinlocation") != -1:
				
				inform_content = act.replace("(informblocknotinlocation b","")
				
				inform_content_split = inform_content.strip().split(" ")

				# DEPENDING ON THE REQ.SCENARIO, we say either Box/cabinet/
				explanation = "Block {0} is NOT in Box {1}".format(inform_content_split[0].replace("(",""),inform_content_split[1].replace(")",""))
				print(explanation)


	# print(natural_language_utter.replace("<EXPLANATION>",explanation))

	# 1. (informblocklocation_b_b1_bx2)


def write_plan_to_file(plan):
	plan_for_regression_file = open('plan_for_regression.txt','w')

	for action in plan:
		plan_for_regression_file.write(action.lower().strip()+"\n")

	plan_for_regression_file.close()

def gen_valid_formula(plan, goal):
	write_plan_to_file(plan)
	#write_goal_in_problem_file(goal)
	#os.system("python ~/REGRESSION_RP_MEP/pdkb-planning/pdkb/planner.py curr_problem_generation.pdkbddl")
	#os.system("python ~/REGRESSION_RP_MEP/pdkb-planning/pdkb/planner.py pdkb-planning/examples/planning/bw4t/EpGoal/prob3.pdkbddl")
	os.system("python ~/REGRESSION_RP_MEP/pdkb-planning/pdkb/planner.py pdkb-planning/examples/planning/grapevine/prob1.pdkbddl")
	

	curr_state_file = open('reg_formula.txt','r')
	raw_lines = curr_state_file.readlines()
	reg_formula = [line.strip() for line in raw_lines if not (line.strip()[0] == 'p' and line.strip()[2] == ' ')]
	reg_formula_formatted = ["(" + pred.replace("_", " ") + ")" for pred in reg_formula]
	reg_formula_formatted_belief = ["[{0}](".format(pred[2]) + pred[4:] for pred in reg_formula_formatted if (pred[1] == 'b' and pred.strip()[3] == ' ')]

	reg_formula_final = reg_formula_formatted_belief + list(set(reg_formula_formatted) - set([pred for pred in reg_formula_formatted if (pred[1] == 'b' and pred.strip()[3] == ' ')]))

	# this might fail if we have in the goal a negation of a belief formula
	reg_formula_final_final = [pred.replace("(!","(not(") + ")" for pred in reg_formula_final if pred.find("!") != -1]

	reg_formula_final_final_final = reg_formula_final_final + list(set(reg_formula_final) - set([pred for pred in reg_formula_formatted if pred.find("!") != -1]))
	

	print(reg_formula_final_final_final)
	return reg_formula_final_final_final

def resolve_disc_charger(req,version):
	# depending on the observation in req, we either do nothing or send an action request depending on the
	# appropriate disc res plan

	# 1 is no ToM and inferring goal from person saying on the phone they're low on battery
	# 2 is no ToM and inferring goal from person plugging in charger
	# 3 is ToM with communication
	# 4 is ToM without communication
	# 5 is discrepancy resolution after 2nd person takes the charger -changing the world
	# 6 is discrepancy resolution after 2nd person takes the charger -informing first human

	if version == '1' or version == '2':
		if req.agent == 'a' and req.event == 'shiftgaze' and req.place == 'q2':
			send_action_request("charger","robot","pickup","")

			send_action_request("","robot","move","den")

			send_action_request("charger","robot","dropOff","den")

	if version == '3':
		if req.agent == 'a' and req.event == 'shiftgaze' and req.place == 'q2':
			send_action_request("charger","robot","pickup","")

			send_action_request("","robot","move","den")

			send_action_request("charger","robot","dropOff","den")

			send_action_request("charger in bedroom","robot","inform","")

	if version == '4':
		if req.agent == 'a' and req.event == 'shiftgaze' and req.place == 'q2':
			send_action_request("","robot","move","bedroom")

			send_action_request("charger","robot","pickup","")

			send_action_request("","robot","move","den")

			send_action_request("charger","robot","dropOff","den")

	if version == '5':
		if req.agent == 'b' and req.event == 'leaveroom' and req.place == 'p1':
			send_action_request("","robot","move","bedroom")

			send_action_request("charger","robot","pickup","")

			send_action_request("","robot","move","livingroom")

			send_action_request("charger","robot","dropOff","livingroom")

	if version == '6':
		if req.agent == 'b' and req.event == 'leaveroom' and req.place == 'p1':
			send_action_request("The charger is not in livingroom, it is in the bedroom",speech/communication)


def resolve_disc_bolander(req,version):
	# depending on the observation in req, we either do nothing or send an action request depending on the
	# appropriate disc res plan

	# 1 is no ToM 
	# 2 is with ToM

	if version == '2':
		if req.agent == 'a' and req.event == 'pickup' and req.object == 'spoon':
			send_action_request("The bowl is not in cabinet2, it is in cabinet1",speech/communication)

def resolve_disc_corridor(req,version):
	# depending on the observation in req, we either do nothing or send an action request depending on the
	# appropriate disc res plan

	# 1 is no ToM 
	# 2 is ToM with world altering disc res
	# 3 is ToM with communicative disc res

	if version == '2':
		if req.agent == 'c' and req.event == 'leaveroom1' and req.place == 'p1':

			send_action_request("","robot","move","doorRoom1")

			send_action_request("doorRoom1","robot","close","")

	if version == '3':
		if req.agent == 'c' and req.event == 'leaveroom1' and req.place == 'p1':

			send_action_request("Alice is in corridor outside room1",speech/communication)

def detect_and_resolve_discrepancies(req):#(req):
	v1 = False # old discrepancy resolution method with checking whether plan is ill formed and doing progression instead of regression
	v2 = True # aligned with 2021 version of disc res paper
	if live:
		#TODO get all agents from problem/domain file go over all agenets and check for discs
		#TODO we assume we have the goal for now but in the future it'll come from goal rec module
		goal = "(holding b b1)"
		if not simplified:
			predicted_plan = predict_agent_plan("b",goal) 
		if v1:
			if is_plan_ill_formed(predicted_plan,goal):
				if aligning_other_agents_beliefs:
					emp_plan = gen_empathetic_plan(goal)
					resolve_disc("b",emp_plan,goal)

					disc_resolving_plan_to_natural_language(emp_plan,goal)

				if changing_world_to_resolve_disc:
					valid_formulae_for_other_agent_plan = gen_valid_formula(predicted_plan, goal)
					valid_formulae_for_other_agent_plan = valid_formulae_for_other_agent_plan.replace("(at charger1 quadrant1)","(chargerInQuadrant quadrant1)")
					goal = "{0}  [b]{1}".format(valid_formulae_for_other_agent_plan)
					run_planner(agent,goal,"prob_template_generation.pdkbddl",[],"encoded_domain.pdkbddl",False)
		if v2:
			# this is aligned with 2021 version of disc res paper. 
			# (1) we predict agent's plan
			# (2) we generate an empathetic plan (that may be identical to the predicted plan if there is some relevant alignment between observer and actor's beliefs)
			# (3) we run the discrepancy resolution algorithm for the predicted plan and the empathetic plan where the disc res goal (in each disjunct) involves a conjunction of the relevant VALID(..) formulae for all the plans in question
			# this way, the observer makes sure there are no discrepancies pertaining to either the empathetic plan or the predicted plan. we don't check if the plan is ill formed because the algorithm will simply return an empty disc res plan if the plan is not ill formed
			# in the simplified version of this we pre compute the regression formulae for empathetic plan and predicted plan and try to compute a disc res plan at every time step
			aligning_other_agents_beliefs = False
			changing_world_to_resolve_disc = True
			if aligning_other_agents_beliefs:
				emp_plan = gen_empathetic_plan(goal)
				resolve_disc("b",predicted_plan,emp_plan,goal)

				disc_resolving_plan_to_natural_language(emp_plan,goal)

			if changing_world_to_resolve_disc:
				if not simplified:
					valid_formulae_for_other_agent_plan = gen_valid_formula(predicted_plan, goal)
					valid_formulae_for_other_agent_plan = valid_formulae_for_other_agent_plan.replace("(at charger1 quadrant1)","(chargerInQuadrant quadrant1)")
					goal = "{0}  [b]{1}".format(valid_formulae_for_other_agent_plan)
					run_planner(agent,goal,"prob_template_generation.pdkbddl",[],"encoded_domain.pdkbddl",False)
					print(parse_plan())
				else:
					# TODO change the  disc res goal after pepper changes the plan that it believes will achieve the human's goal (the assistive solution). we do this manually for now
					if (req.scenario == 'charger' and req.version == 'x') or (req.scenario == 'corridor' and req.version == 'x') or (not newEmpatheticPlan):
						plan = run_planner(-1,"(disc_resolution)","prob_template_generation.pdkbddl",[],"NO_COMM_domain_regression_based_disc_res.pdkbddl",False)
						send_pepper_plan_to_action_server(parse_plan(),"")
					else:
						plan = run_planner(-1,"(disc_resolution) (disc_resolution_two)","prob_template_generation.pdkbddl",[],"domain_regression_based_disc_res.pdkbddl",False)
						send_pepper_plan_to_action_server(parse_plan(),"")
						# disc_resolving_plan_to_natural_language(parse_plan(),"")
						# print(parse_plan())

	else:
		if req.scenario == 'charger':
			resolve_disc_charger(req,req.scenarioVersion)
		if req.scenario == 'bolander':
			resolve_disc_bolander(req,req.scenarioVersion)
		if req.scenario == 'corridor':
			resolve_disc_corridor(req,req.scenarioVersion)


def OLD_generate_knowhow_preserving_plan():
	goal = "(holding b b1)"
	other_agent_plan = "..." # e.g., take tool from `orig' location
	other_agent_goal = "..." 
	# TODO to generalize this, we need to go over all agents' possible goals and all posisble plans
	# for now, we assume one agent, one goal, and one possible plan
	valid_formulae_for_other_agent_plan = gen_valid_formula(other_agent_plan, other_agent_goal)
	plan = gen_knowhow_pres_plan_no_disc(goal,valid_formulae_for_other_agent_plan) 
	update_problem_file(plan)
	if plan != null: #or however we want to check whether no plan was found
		return plan
	else:
		emp_plan = gen_empathetic_plan(otherAgentGoal)
		resolve_disc("b",emp_plan,otherAgentGoal)
		# we might need to generate another disc resolving plan that explains why other_agent_plan is no longer valid
		return parse_plan() 

def detect_and_resolve_discrepancies_kitchen(plan):
	send_action_request("The bowl is not in the dishwasher, it is in the bedroom",speech/communication)

def run_mepk_planner(epddl_file_name):
	os.system("./mepk {} > predicted_plan.txt".format(epddl_file_name))
	# read plan from discrimination_plan.txt
	return ["pepper_move_near_cabinet","check_cabinet_spoon","check_cabinet_bowl"]
	

def discriminate_between_plans(action_tree,past_observations,req):

	# version 1 for kitchen is no-ToM case where pepper doesn't intervene

	discrimination_plan = ""

	if 0: #req.event == 'move' or req.event == 'open' or req.scenarioVersion == '1':
		return 0

	# here we look at past_observations (we assume that obs include actions like 'check_cabinet_spoon')
	# for each observation in sequence, we find the first occurance of it in the tree
	# then we take the all the branches in the cartesian product of the sensing results of the observations
	# so if we have, for example, check_cabinet_spoon and check_cabinet_bowl then the cartesian product is
	# [bowl&spoon, !bowl&spoon, bowl&!spoon, !bowl&!spoon] and these correspond to four subtrees between which we discriminate
	# to get sensing results and to formulate the epistemic goal we go to EPDDL domain and check what the obs_pos and obs_neg of the actions are
	if live:
		if not simplified:
			plans_to_be_discriminated_between, product_of_sensing_results = get_branches_from_tree_given_observations(action_tree,past_observations)

		# the epistemic goal is a disjunction over the observer's beliefs about actor's beliefs about each sensing result in product
			discrimination_epistemic_goal = "(or "
			for result in product_of_sensing_results:
				discrimination_epistemic_goal += "(K_pepper(K_human {0} )) "

			discrimination_epistemic_goal += ")"
	
		run_mepk_planner("kitchen_active_epr.epddl")
		discrimination_plan = parse_plan_mepk()


	observations = [] #REINSTATE THIS after integration with Ruthrash execute_discrimination_plan(discrimination_plan)


	# # after executing the discr plan, we need to check what the observations are from pepper's sensing
	# # we need a global object that keeps the growing sequence of observations
	# for enumerate i, sensing_result in product_of_sensing_results:
	# 	if sensing_result in global_observation_sequence:
	# 		return plans_to_be_discriminated_between[i]

	# alternatively, Just make four conditionals
    # And check which one of them is satisfied by the observations
    
	need_for_disc_res = False

    # if the domain includes special elimination actions then the plan will include e.g., elimination_action_1
    # and we will be able to tell which tree is left 
	if "in_bowl_cabinet" in observations and "in_spoon_cabinet" in observations:
		cmd = "printf 'o0\ne0+\ne1+' | ./test simplified_kitchen_active_epr.epddl > output.txt | pkill -f './test simplified_kitchen_active_epr.epddl'"
		need_for_disc_res = False
	if "in_bowl_cabinet" in observations and "not_in_spoon_cabinet" in observations:
		cmd = "printf 'o0\ne0-\ne1+' | ./test simplified_kitchen_active_epr.epddl > output.txt | pkill -f './test simplified_kitchen_active_epr.epddl'"
		need_for_disc_res = False
	if "not_in_bowl_cabinet" in observations and "in_spoon_cabinet" in observations:
		cmd = "printf 'o0\ne0+\ne1-' | ./test simplified_kitchen_active_epr.epddl > output.txt | pkill -f './test simplified_kitchen_active_epr.epddl'"
		need_for_disc_res = True
	if "not_in_bowl_cabinet" in observations and "not_in_spoon_cabinet" in observations:
		cmd = "printf 'o0\ne0-\ne1-' | ./test simplified_kitchen_active_epr.epddl > output.txt | pkill -f './test simplified_kitchen_active_epr.epddl'"
		need_for_disc_res = True

	timeoutSeconds = 1
	try:
		cmd = "printf 'o0\ne0-\ne1-' | ./test simplified_kitchen_active_epr.epddl > output.txt | pkill -f './test simplified_kitchen_active_epr.epddl'"
		subprocess.check_output(cmd, shell=True, timeout=timeoutSeconds)
	except:
		a=0

	print("===========================================\n")
	os.system("cat output.txt")
	print("===========================================\n")
	return need_for_disc_res

def execute_discrimination_plan(discrimination_plan):
	observations = []
	if live:
		if not simplified:
			for action in discrimination_plan:
				observations.append(send_action_request(action.object,action.agent,action.action))
		else:
			observations.append(send_action_request("","robot","move","nearCabinet",""))
	else:
		observations.append(send_action_request("","robot","move","nearCabinet",""))

		observations.append(send_action_request("cabinet","robot","observe",""))


	return observations


def encode_as_disjunctive_goal(valid_formulae_for_set_of_other_agent_plans):
	for valid_formula in valid_formulae_for_set_of_other_agent_plans:
		# each action's precondition (or condition of conditional effect) should be
		# (and valid_formula [i]valid_formula) and the effect should be (and khowPreservingGoal)
		add_special_action_for_disjunctive_goal(valid_formula)

def generate_knowhow_preserving_plan(goal):
	if not simplified:
		#goal = "(holding b b1)"
		set_of_other_agent_plans = "..." # e.g., take tool from `orig' location
		other_agent_goal = "..." 
		# TODO to generalize this, we need to go over all agents' possible goals and all posisble plans
		# for now, we assume one agent, one goal, and one possible plan
		valid_formulae_for_set_of_other_agent_plans = [gen_valid_formula(other_agent_plan, other_agent_goal) for other_agent_plan in set_of_other_agent_plans]
		# TODO in the future, we would want to encode the set of plans etc
		# but for now we can just create the domains with the special actions in place since the set of plans won't change
		#encode_as_disjunctive_goal(valid_formulae_for_set_of_other_agent_plans)
		plan = gen_knowhow_pres_plan_no_disc(goal,"(khowPreservingGoal)") 
		if plan != null: #or however we want to check whether no plan was found
			return plan

		else:
			update_problem_file(plan)
			emp_plan = gen_empathetic_plan(otherAgentGoal)
			resolve_disc("b",emp_plan,otherAgentGoal)
			# we might need to generate another disc resolving plan that explains why other_agent_plan is no longer valid
			return parse_plan() 
	else:
		if 0:#req.scenario == 'charger' and req.version == 'x':
			plan = run_planner(-1,"(khowPreservingGoal) {0}".format(goal),"prob_template_generation.pdkbddl",[],"NO_COMM_domain_template_for_generation_of_pepper_behavior.pdkbddl",False)
		else:
			plan = run_planner(-1,"(khowPreservingGoal) {0}".format(goal),"prob_template_generation.pdkbddl",[],"domain_template_for_generation_of_pepper_behavior.pdkbddl",False)
		send_pepper_plan_to_action_server(parse_plan(),"")
	

def goal_detected(req):
	if req.scenario == 'bolander_spoon_cabinet':
		if req.event == 'takeobjectoutofcabinet' and req.object.find("spoon") != -1:
			return True
		else:
			return False
	return True

def format_observations(req):
	# either standardize all actions' parameter order so that agent appears before objects that appears before ..
	# or have a number of conditionals here that check req.event and construct the observation according to it
	observed_actions = []
	if req.object == '':
		observed_actions.append("(" + req.event + "_" + req.agent + "_" + req.place + ")")
	else:
		observed_actions.append("(" + req.event + "_" + req.agent + "_" + req.object + "_" + req.place + ")")

	if req.event == 'shiftgaze':
		observed_actions.append('(update_vision_based_beliefs)')
		observed_actions.append('(shiftgaze_reset)')

	if req.event == 'leaveRoom' and req.scenario == 'corridor':
		# do the following unless event server can send the leaveroomAndStayOutside1 observation after seeing that Eve left room but stayed outside
		observed_actions = []
		observed_actions.append('(leaveroomAndStayOutside1'.lower() + "_" + req.agent + req.place + ")")

	if req.event == 'pickupcharger':
		newEmpatheticPlan = True

	return observed_actions


if ROS:
	def eventRequestCB(req):
		response_ = EventServerResponse()
		out_ = "Event observed: Object is {}, agent is {},event is {}, place is {}".format(req.object, req.agent,req.event, req.place)
		rospy.loginfo(out_)    

		observed_actions = format_observations(req)

		update_problem_file(observed_actions)

		# either add a sympathetic attribute to req or just have a bigger conditional with the specifc scenario + version combos that are the control condition
		if req.sympathetic:
			# check this using req.scenario and req.version
			if CHARGER_CONTROL_CONDITION: 
				# we should get the non-khow preserving plan
				generate_knowhow_preserving_plan("(chargerInQuadrant q9)")

			return response_

		if req.scenario == 'kitchen':
			# ideally, here we would check whether the observations we have so far leave us with a number of 
			# plan hypotheses where some of those hypotheses require intervention (disc res)
			# and some don't. in this case (as is the case in the kitchen scenario) we should attempt to
			# discriminate between the different plan hypotheses and then act accordingly 
			recognized_plan = discriminate_between_plans(0,0,0)#(kitchen_action_tree,past_observations)

			if recognized_plan:
				detect_and_resolve_discrepancies_kitchen(recognized_plan)

		else:
			if goal_detected(req):
				detect_and_resolve_discrepancies(req)
				# NOTE when there is first a discrepancy between pepper's beliefs and the beliefs of other agents. when providing an explanation to 
				# an agent with whom pepper has a discrepancy, pepper can tell that agent what has been observed in the span since there was no discrepancy
				# in our case, pepper will tell agent b that pickup(b,b1,bx1) putIn(b,b1,bx2) enterroom(a,p1) were observed 

		return response_

	# def callback(data):

	# 	rospy.init_node('planner OR tom module')
	#     rospy.wait_for_service('/action_server')
	#     rospy.wait_for_service('/event_server')
	#     scenario1_v1_service = rospy.ServiceProxy('/event_server', EventServer)
	#     event_req = EventServerRequest()


	#     rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

	#     update_problem_file(data.data)#,"....","curr_problem.pddl")

	#     if goal_detected(data.data):
	#     	detect_and_resolve_discrepancies(req)
	#     	# NOTE when there is first a discrepancy between pepper's beliefs and the beliefs of other agents. when providing an explanation to 
	#     	# an agent with whom pepper has a discrepancy, pepper can tell that agent what has been observed in the span since there was no discrepancy
	#     	# in our case, pepper will tell agent b that pickup(b,b1,bx1) putIn(b,b1,bx2) enterroom(a,p1) were observed 

	def listener():

	    # In ROS, nodes are uniquely named. If two nodes with the same
	    # name are launched, the previous one is kicked off. The
	    # anonymous=True flag means that rospy will choose a unique
	    # name for our 'listener' node so that multiple listeners can
	    # run simultaneously.
	    rospy.init_node('listener', anonymous=True)

	    service_ = rospy.Service('/event_server', EventServer, eventRequestCB )

	    # spin() simply keeps python from exiting until this node is stopped
	    rospy.spin()


	if __name__ == '__main__':
	    listener()



# gen_valid_formula(["pickup_b_blue_2_roomc2"], "")
# gen_valid_formula(["share_a_a_l1"], "")

# while(1):
# 	0

req = 0

#=================================BOLANDER SCENARIO
# first human is agent b, second human is agent a (first human leaves the room and has the false belief)
# goal is (cereal_made) and the plan for this is [pickUpSpoon, pickUpBowlcabinet2, pickUpMilk, pickUpCereal, pourCerealToBowl, pourMilkToBowl]
# plan is invalid because the regression formula for pickUpBowlcabinet2 is in(bowl,cabinet2) which does not hold 
# because the first human took the bowl from cabinet2, used it, and transferred it to cabinet1
# in the blocks and boxes encoding, block b1 is the bowl and it is picked up from bx1 (cabinet2) and put in bx2 (cabinet1)

if global_current_domain == 'bolander':
	curr_state_file = open('curr_state.txt','w')
	curr_state_file.write(initial_state_bolander)

	curr_state_file.close()

	print("Initial state:\n")
	print("====================\n")
	print(initial_state_bolander)
	print("====================\n")
	  
	observed_actions = ['(leaveRoom_b_p1)']
	update_problem_file(observed_actions)

	print("AFTER (leaveRoom_b_p1)")
	detect_and_resolve_discrepancies(req)

	observed_actions = ['(pickUpBlock_a_b1_bx1_p1)']
	update_problem_file(observed_actions)

	print("AFTER (pickUpBlock_a_b1_bx1_p1)")
	detect_and_resolve_discrepancies(req)

	observed_actions = ['(putBlockInBox_a_b1_bx2_p1)']
	update_problem_file(observed_actions)


	print("AFTER (putBlockInBox_a_b1_bx2_p1)")
	detect_and_resolve_discrepancies(req)

	observed_actions = ['(enterRoom_b_p1)']
	update_problem_file(observed_actions)

	print("AFTER (enterRoom_b_p1)")

	detect_and_resolve_discrepancies(req)

	# the above produces the following which is what we want because there is no need for Pepper to resolve discrepancies
	# until the person comes back into the room

	# AFTER (leaveRoom_b_p1)
	# } 
	# AFTER (pickUpBlock_a_b1_bx1_p1)
	# } 
	# AFTER (putBlockInBox_a_b1_bx2_p1)
	# } 
	# AFTER (enterRoom_b_p1)
	# (informblocklocation b b1 bx2) (discresolutionspecialactionforplanb2) (informblocknotinlocation b b1 bx1) (discresolutionspecialaction1) } 

#=================================BOLANDER SCENARIO (SPOON AND CABINET VARIANT)

if global_current_domain == 'bolander_spoon_cabinet':

	curr_state_file = open('curr_state.txt','w')
	curr_state_file.write(initial_state_bolander_spoon_cabinet)

	curr_state_file.close()

	print("Initial state:\n")
	print("====================\n")
	print(initial_state_bolander_spoon_cabinet)
	print("====================\n")
	  
	observed_actions = ['(opencabinet_b_cabinet2_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(putobjectincabinet_b_bowl1_cabinet2_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(closecabinet_b_cabinet2_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(leaveRoom_b_p1)']
	update_problem_file(observed_actions)

	detect_and_resolve_discrepancies(req)

	# after this observation we get an empty disc res plan

	observed_actions = ['(opencabinet_a_cabinet2_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(takeobjectoutofcabinet_a_bowl1_cabinet2_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(closecabinet_a_cabinet2_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(opencabinet_a_cabinet1_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(putobjectincabinet_a_bowl1_cabinet1_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(closecabinet_a_cabinet1_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(leaveRoom_a_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(enterRoom_b_p1)']
	update_problem_file(observed_actions)

	observed_actions = ['(opencabinet_b_cabinet3_p1)']
	update_problem_file(observed_actions)



	observed_actions = ['(takeobjectoutofcabinet_b_spoon1_cabinet3_p1)']
	update_problem_file(observed_actions)

	# print("AFTER (takeobjectoutofcabinet_b_bowl1_cabinet3_p1)")
	detect_and_resolve_discrepancies(req)

	# this is the disc res plan after the last observation

	#(informspoonnotinlocation b spoon1 cabinet2) (discresolutionspecialaction1) (informspoonlocation b spoon1 cabinet1) (discresolutionspecialactionforplanb2)



#=================================================================
#================================ phone charger scenario
# first human is agent a, second human is agent b
# goal is (and (charged phone_a))

if global_current_domain == 'charger' or global_current_domain == 'charger_kHow':

	curr_state_file = open('curr_state.txt','w')
	curr_state_file.write(initial_state_charger)

	curr_state_file.close()

	print("Initial state:\n")
	print("====================\n")
	print(initial_state_charger)
	print("====================\n")
	  
	# observed_actions = ['(plugincharger_a_q1)']
	# update_problem_file(observed_actions)

	observed_actions = ['(shiftgaze_reset_a)']
	update_problem_file(observed_actions)

	observed_actions = ['(shiftgaze_a_q1)']
	update_problem_file(observed_actions)

	observed_actions = ['(update_vision_based_beliefs)']
	update_problem_file(observed_actions)

	# observed_actions = ['(chargerAxiom_)']
	# update_problem_file(observed_actions)

	observed_actions = ['(shiftgaze_reset_a)']
	update_problem_file(observed_actions)

	observed_actions = ['(shiftgaze_a_q2)']
	update_problem_file(observed_actions)

	observed_actions = ['(update_vision_based_beliefs)']
	update_problem_file(observed_actions)

	if global_current_domain == 'charger_kHow':
		#Pepper receives message from human and is tasked with bringing a charger to room3 (other charger is in room2)
		generate_knowhow_preserving_plan("(chargerInQuadrant q9)") #q9 is in room3

	else:

		#SECOND HUMAN COMING INTO ROOM AND TAKING CHARGER

		observed_actions = ['(enterroom1_b_p1)']
		update_problem_file(observed_actions)

		# if the following two observations are sent to pepper one after the other then
		# we might generate and try to execute another non-empty disc res plan because we haven't yet finished executing the previous one
		# we might want a flag that stops the computational of new disc res plans while a current disc res plan is being executed

		observed_actions = ['(pickupcharger_b_q1)']
		update_problem_file(observed_actions)

		observed_actions = ['(leaveroom1_b_p1)']
		update_problem_file(observed_actions)

		detect_and_resolve_discrepancies(req) 
		# we can simplify this by simply calling the planner with the goal (chargerInQuadrant q1) [b](chargerInQuadrant q1) 
		# initially, [b](chargerInQuadrant q1)  holds but (chargerInQuadrant q1) doesn't
		# so the planner will come up with a plan to move robot from q1 to q5, robot picking up charger, 
		# and finally robot returning to q1 and droppping off the charger there

#=================================================================
# corridor domain
# first human is agent a, second human is agent b, and third human is agent c
# goal is (and [a][b](secret) [a]![c](secret))
if global_current_domain == 'corridor':
	curr_state_file = open('curr_state.txt','w')
	curr_state_file.write(initial_state_corridor_scenario)

	curr_state_file.close()

	print("Initial state:\n")
	print("====================\n")
	print(initial_state_corridor_scenario)
	print("====================\n")

	# the first observation is person c entering the room

	observed_actions = ['(leaveroomAndStayOutside1_c)'] # to update KB, we observe leaveroomAndStayOutside1
	update_problem_file(observed_actions)

	if 1:
		detect_and_resolve_discrepancies(req) 

#=================================================================
#=================================================================
# kitchen domain

if global_current_domain == 'kitchen':
	curr_state_file = open('curr_state.txt','w')
	curr_state_file.write(initial_state_kitchen)
	curr_state_file.close()

	observed_actions = ['(openCabinet_a)'] # the human opens one of the cabinets in the kitchen (and is assumed to look into it, so the effects of openCabinet also encode an observer (pepper in this case) reasoning that the human knows whether there's a spoon and bowl in cabinet)
	update_problem_file(observed_actions)

	discriminate_between_plans(0,0,0)
	# observed_actions = ['(check_cabinet_spoon(human))'] 
	# update_problem_file(observed_actions)
	# observed_actions = ['(check_cabinet_bowl(human))'] 
	# update_problem_file(observed_actions)
	# observed_actions = ['(moveTo_Dishwasher_a)'] # the human moves from cabinet to dishwasher
	# update_problem_file(observed_actions) 

#=================================================================


# goal = "(holding b b1)"
# predicted_plan = predict_agent_plan("b",goal)
# # print(predicted_plan)

# if is_plan_ill_formed(predicted_plan,goal):
# 	emp_plan = gen_empathetic_plan(goal)
# 	resolve_disc("b",emp_plan,goal)



# # observed_actions = ['(putBlockInBox_a_b1_bx2_p1)']
# # update_problem_file(observed_actions,"....","curr_problem.pddl")
