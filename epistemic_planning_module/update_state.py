import os

# import rospy
# from std_msgs.msg import String
# 
initial_state = "(at a p1) (at b p1) [a](at a p1) [a](at b p1) [b](at a p1) [b](at b p1) (atBox bx1 p1) (atBox bx2 p1) (atBox bx3 p1) [a](atBox bx1 p1) [a](atBox bx2 p1) [a](atBox bx3 p1) [b](atBox bx1 p1) [b](atBox bx2 p1) [b](atBox bx3 p1) (in b1 bx1) (in b2 bx2) (in b3 bx3) [a](in b1 bx1) [a](in b2 bx2) [a](in b3 bx3) [b](in b1 bx1) [b](in b2 bx2) [b](in b3 bx3) [a](!holding a b1) [b](!holding a b1) (dummy) (atRobot p1)"
ROS = True

def run_planner(agent,goal,template_file,plan_to_execute,domain_file_name,validation):
	curr_state_file = open('curr_state.txt','r')
	raw_lines = curr_state_file.readlines()
	lines = [line.strip() for line in raw_lines]
	curr_state = lines[0]

	if validation:
		problem_file = open("curr_problem.pdkbddl", "w")
		template_file = open("prob_template.pdkbddl", "r")
	else:
		problem_file = open('curr_problem_generation.pdkbddl','w')
		template_file = open("prob_template_generation.pdkbddl", "r")
		

	raw_lines = template_file.readlines()
	lines = [line.strip() for line in raw_lines]

	for line in lines:
		# print(line)
		if agent == -1:
			if validation:
				problem_file.write(line.replace("<TEMPLATE>","(:plan " + plan_to_execute + " )").replace("<INIT_TEMPLATE>",curr_state).replace("<GOAL_TEMPLATE>","{0}".format(goal)).replace("include:domain.pdkbddl","include:{0}".format(domain_file_name)))
				problem_file.write('\n')
			else:
				problem_file.write(line.replace("<TEMPLATE>","").replace("<INIT_TEMPLATE>",curr_state).replace("<GOAL_TEMPLATE>","{0}".format(goal).replace("include:domain.pdkbddl","include:{0}".format(domain_file_name))))
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
		os.system("python pdkb/planner.py curr_problem.pdkbddl --keep-files > final_state.txt")
	else:
		os.system("python pdkb/planner.py curr_problem_generation.pdkbddl --keep-files > predicted_plan.txt")

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


	plan_to_execute = ""
	for act in observed_actions:
		plan_to_execute += (act + " ")


	run_planner(-1,"(dummy)","prob_template.pdkbddl",plan_to_execute,"domain.pdkbddl",validation=True)
	

	next_state = parse_final_state()


	curr_state = next_state

	curr_state_file = open('curr_state.txt','w')
	curr_state_file.write(curr_state)

	curr_state_file.close()


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
	

def disc_resolving_plan_to_natural_language(emp_plan,goal):
	plan = parse_plan()
	actions = plan.split(" ")

	#natural_language_utter = "The plan {0} will fail to achieve goal {1} since"
	natural_language_utter = "If your goal is {0}, then you better do {1} because <EXPLANATION>".format(goal,emp_plan)

	

	for act in actions:
		if act.find("inform") != -1:
			if act.find("blocklocation") != -1:
				inform_content = act.replace("informblocklocation_b_","")
				inform_content_split = inform_content.split("_")


				explanation = "Block {0} is in Box {1}".format(inform_content_split[0].replace("(",""),inform_content_split[1].replace(")",""))


	print(natural_language_utter.replace("<EXPLANATION>",explanation))

	# 1. (informblocklocation_b_b1_bx2)

def detect_and_resolve_discrepancies():
	#TODO get all agents from problem/domain file go over all agenets and check for discs
	#TODO we assume we have the goal for now but in the future it'll come from goal rec module
	goal = "(holding b b1)"
	predicted_plan = predict_agent_plan("b",goal) 
	if is_plan_ill_formed(predicted_plan,goal):
		emp_plan = gen_empathetic_plan(goal)
		resolve_disc("b",emp_plan,goal)

		disc_resolving_plan_to_natural_language(emp_plan,goal)


# if ROS:
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

    update_problem_file(data.data)#,"....","curr_problem.pddl")

    # if goal_detected(data.data):
    # 	detect_and_resolve_discrepancies()
    
    	# NOTE when there is first a discrepancy between pepper's beliefs and the beliefs of other agents. when providing an explanation to 
    	# an agent with whom pepper has a discrepancy, pepper can tell that agent what has been observed in the span since there was no discrepancy
    	# in our case, pepper will tell agent b that pickup(b,b1,bx1) putIn(b,b1,bx2) enterroom(a,p1) were observed 

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()

# curr_state_file = open('curr_state.txt','w')
# curr_state_file.write(initial_state)

# curr_state_file.close()
  
# observed_actions = ['(leaveRoom_b_p1)']
# update_problem_file(observed_actions)

# observed_actions = ['(pickUpBlock_a_b1_bx1_p1)']
# update_problem_file(observed_actions)

# observed_actions = ['(putBlockInBox_a_b1_bx2_p1)']
# update_problem_file(observed_actions)

# observed_actions = ['(enterRoom_b_p1)']
# update_problem_file(observed_actions)

# detect_and_resolve_discrepancies()

# goal = "(holding b b1)"
# predicted_plan = predict_agent_plan("b",goal)
# # print(predicted_plan)

# if is_plan_ill_formed(predicted_plan,goal):
# 	emp_plan = gen_empathetic_plan(goal)
# 	resolve_disc("b",emp_plan,goal)



# # observed_actions = ['(putBlockInBox_a_b1_bx2_p1)']
# # update_problem_file(observed_actions,"....","curr_problem.pddl")
