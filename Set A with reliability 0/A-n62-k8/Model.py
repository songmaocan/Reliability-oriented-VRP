from Read_data import Read_txt
import copy
class Method:
    def __init__(self):
        based_profit=0
        data=Read_txt(based_profit)
        self.node_list, self.link_list, self.g_number_of_nodes, self.capacity,self.origin,self.destination=data.generate_links()
        self.g_number_of_vehicles=8
        self.rpo=1

        #DP parameter
        self.Best_K_Size = 100
        self.g_number_of_time_intervals = 1000
        self.g_ending_state_vector = [None] * (self.g_number_of_vehicles+1)
        self.big_M=100

        self.g_time_dependent_state_vector = []
        for v in range(self.g_number_of_vehicles+1):
            self.g_time_dependent_state_vector.append([None] * self.g_number_of_time_intervals)

        #iteration
        self.record_profit=[]
        self.served_times=[]
        self.un_served=[]
        self.repeat_served=[]
        self.iteration_times=100
        self.vehicle_node_sequence=[]
        self.local_LB = [0] * self.iteration_times
        self.local_UB = [0] * self.iteration_times
        self.global_LB = [-10000] * self.iteration_times
        self.global_UB = [10000] * self.iteration_times

    def solving(self):
        self.optimal_iteration_for_UB=0
        for i in range(self.iteration_times):
            self.un_served.append([])
            self.repeat_served.append([])
            self.vehicle_node_sequence.append([])
            self.record_profit.append([])
            print("iteration: {}".format(i+1))
            self.served_times.append([0]*self.g_number_of_nodes)
# step1:UB generation
            if i!=0:
                self.served_times[i]=copy.deepcopy(self.served_times[i-1])

            for v in range(self.g_number_of_vehicles):

                if self.g_ending_state_vector[v]!=None:
                    node_serving_state = self.g_ending_state_vector[v].VSStateVector[0].node_serving_state
                    for node_id in range(1,self.g_number_of_nodes-1):
                        self.served_times[i][node_id]-=node_serving_state[node_id]

                for node_id in range(1,self.g_number_of_nodes-1):
                    multiplier=self.node_list[node_id].base_profit_for_lagrangian
                    self.node_list[node_id].base_profit_for_searching=multiplier+self.served_times[i][node_id]*self.rpo-self.rpo/2

                self.g_dynamic_programming_algorithm(v,1)
                self.local_UB[i] += self.g_ending_state_vector[v].VSStateVector[0].Primal_Label_cost
                self.vehicle_node_sequence[i].append(self.g_ending_state_vector[v].VSStateVector[0].node_seq)

                node_serving_state = self.g_ending_state_vector[v].VSStateVector[0].node_serving_state
                for node_id in range(1, self.g_number_of_nodes - 1):
                    self.served_times[i][node_id] += node_serving_state[node_id]

            Flag=0
            for node_id in range(1,self.g_number_of_nodes-1):
                if self.served_times[i][node_id]==0:
                    self.un_served[i].append(node_id)
                    self.local_UB[i]+=self.big_M

                if self.served_times[i][node_id]>1:
                    self.repeat_served[i].append(node_id)
                    Flag=1


            print(self.served_times[i])
            # if Flag==0:
            #     print("All customers are covered!")
            #     print(self.local_UB[i])

#step 2: LB
            self.g_dynamic_programming_algorithm(self.g_number_of_vehicles, 2)

            # for v in range(self.g_number_of_vehicles):
            #     self.local_LB[i]+=min(self.g_ending_state_vector[self.g_number_of_vehicles].VSStateVector[v].Label_cost_for_lagrangian,0)

            self.local_LB[i] += min(self.g_ending_state_vector[self.g_number_of_vehicles].VSStateVector[0].Label_cost_for_lagrangian, 0)*self.g_number_of_vehicles

            for node_id in range(1,self.g_number_of_nodes-1):
                self.local_LB[i]-=self.node_list[node_id].base_profit_for_lagrangian

            #update global lower bound and upper bound
            if i==0:
                self.global_LB[i]=self.local_LB[i]
                if Flag == 0:
                    self.global_UB[i] = self.local_UB[i]
                    self.optimal_iteration_for_UB=i
            else:
                self.global_LB[i] = max(self.local_LB[i],self.global_LB[i-1])

                if Flag==0:
                    self.global_UB[i] = min(self.local_UB[i],self.global_UB[i-1])
                    self.optimal_iteration_for_UB=i
                else:
                    self.global_UB[i] = self.global_UB[i-1]

#step 3:multiplier updating
            for node_id in range(1,self.g_number_of_nodes-1):
                self.record_profit[i].append(self.node_list[node_id].base_profit_for_lagrangian)
                self.node_list[node_id].base_profit_for_lagrangian+=self.rpo*(self.served_times[i][node_id]-1)
                if self.node_list[node_id].base_profit_for_lagrangian>0:
                    self.node_list[node_id].base_profit_for_lagrangian=0

            if i >= 30:
                if (len(self.un_served[i]) + len(self.repeat_served[i])) ** 2 > 0.25 * (len(self.un_served[i - 1]) + len(self.repeat_served[i - 1])) ** 2:
                    self.rpo += 2
                if (len(self.un_served[i]) + len(self.repeat_served[i])) ** 2 == 0:
                    self.rpo = 1

#step 4: terminal conditions
            # gap=(self.global_UB[i]-self.global_LB[i])/self.global_UB[i]
            # if gap<0.02:
            #     self.consumed_iteration=i+1
            #     break
            self.consumed_iteration = self.iteration_times


    def g_dynamic_programming_algorithm(self, vehicle_id, Flag):
        for t in range(self.g_number_of_time_intervals):
            self.g_time_dependent_state_vector[vehicle_id][t] = C_time_indexed_state_vector()
            self.g_time_dependent_state_vector[vehicle_id][t].Reset()
            self.g_time_dependent_state_vector[vehicle_id][t].current_time = t

        self.g_ending_state_vector[vehicle_id] = C_time_indexed_state_vector()
        max_time_interval = 0
        # 1.Initial state for original depot
        new_element = CVSState()
        new_element.current_node_id = self.origin
        new_element.remaining_capacity = self.capacity
        new_element.node_seq.append(self.origin)
        new_element.node_serving_state = [0] * self.g_number_of_nodes
        self.g_time_dependent_state_vector[vehicle_id][0].update_state(new_element, Flag)
        for t in range(self.g_number_of_time_intervals):
            for index in range(min(len(self.g_time_dependent_state_vector[vehicle_id][t].VSStateVector), self.Best_K_Size)):
                pElement = self.g_time_dependent_state_vector[vehicle_id][t].VSStateVector[index]
                from_node_id = pElement.current_node_id  # ??????ID
                from_node = self.node_list[from_node_id]  # ??????
                for i in range(from_node.outbound_size):  # ?????????
                    to_node_id = from_node.outbound_nodes_list[i]  # ?????????????????????????????????ID
                    to_node = self.node_list[to_node_id]  # ?????????
                    link_to = from_node.outbound_links_list[i]  # ????????????
                    if link_to.distance == 0:
                        next_time = t + 1
                    else:
                        next_time = t + link_to.distance

                    if next_time > self.g_number_of_time_intervals - 1:
                        continue
                    # Case 1: to_node is the destination
                    if to_node_id == self.destination:
                        new_element = CVSState()
                        new_element.my_copy(pElement)
                        # new_element.current_node_id = to_node_id
                        # new_element.remaining_capacity -= to_node.demand
                        new_element.node_seq.append(to_node_id)
                        # new_element.node_serving_state[to_node_id] = 1
                        new_element.Calculate_Label_Cost(to_node, link_to)
                        self.g_ending_state_vector[vehicle_id].update_state(new_element, Flag)

                    # Case 2: to_node is not the destination
                    if to_node_id != self.destination:
                        # check 1: vehicle capacity
                        if pElement.remaining_capacity < to_node.demand:
                            continue
                        # check 2: if the to node is served
                        node_serving_state = pElement.node_serving_state
                        if node_serving_state[to_node_id] == 1:
                            continue

                        new_element = CVSState()
                        new_element.my_copy(pElement)
                        new_element.current_node_id = to_node_id
                        new_element.remaining_capacity -= to_node.demand
                        new_element.node_seq.append(to_node_id)
                        new_element.node_serving_state[to_node_id] = 1
                        new_element.Calculate_Label_Cost(to_node, link_to)
                        self.g_time_dependent_state_vector[vehicle_id][next_time].update_state(new_element, Flag)
                        if next_time > max_time_interval:
                            max_time_interval = next_time
            # check
            if max_time_interval <= t:
                break
        self.g_ending_state_vector[vehicle_id].Sort(Flag)

    def output_to_file(self,spend_time):
        #multiplier
        with open("Output_multipliers.csv","w") as fl:
            fl.write("iteration,")
            for node_id in range(1,self.g_number_of_nodes-1):
                str_="node-"+str(node_id)+","
                fl.write(str_)
            fl.write("\n")

            for i in range(self.consumed_iteration):
                multiplier_list=self.record_profit[i]
                fl.write(str(i+1)+",")
                for multiplier in multiplier_list:
                    fl.write(str(multiplier)+",")
                fl.write("\n")

        #bounds
        with open("Output_gap.csv","w") as fl:
            fl.write("iteration,lb,ub,gap,flag\n")
            for i in range(self.consumed_iteration):
                fl.write(str(i + 1) + ",")
                fl.write(str(self.global_LB[i])+",")
                fl.write(str(self.global_UB[i]) + ",")
                gap=round((self.global_UB[i]-self.global_LB[i])/self.global_UB[i],3)
                fl.write(str(gap) + ",")
                Flag = 0
                for node_id in range(1, self.g_number_of_nodes - 1):
                    if self.served_times[i][node_id] == 0:
                        Flag = 1
                    if self.served_times[i][node_id] > 1:
                        Flag = 1
                if Flag==0:
                    fl.write("Yes\n")
                else:
                    fl.write("No\n")
            fl.write("running time {} seconds".format(round(spend_time)))

        #solution
        with open("Output_solution.csv","w") as fl:
            fl.write("Optimal primal value: {}\n".format(self.global_UB[-1]))
            fl.write("Optimal dual value: {}\n".format(self.global_LB[-1]))
            gap=(self.global_UB[-1]-self.global_LB[-1])/self.global_UB[-1]
            fl.write("gap: {}\n".format(round(gap,4)))
            fl.write("optimal iteration: {}\n".format(self.optimal_iteration_for_UB))
            fl.write("Primal optimal solution: ")
            node_seq=self.vehicle_node_sequence[self.optimal_iteration_for_UB]
            fl.write(str(node_seq)+"\n")

            fl.write("Dual optimal solution: ")
            Multiplier_list=self.record_profit[self.optimal_iteration_for_UB]
            fl.write(str(Multiplier_list) + "\n")
            fl.write("running time {} seconds".format(round(spend_time)))

class CVSState:
    def __init__(self):
        # node state
        self.current_node_id = 0
        self.remaining_capacity = 0
        # path state
        self.node_seq = []
        self.node_serving_state = []
        # path costs
        self.Primal_Label_cost = 0
        self.Label_cost_for_lagrangian = 0
        self.Label_cost_for_searching = 0

    def generate_string_key(self):
        key = self.current_node_id
        return key

    def my_copy(self, pElement):
        # node state
        self.current_node_id = copy.copy(pElement.current_node_id)
        self.remaining_capacity = copy.copy(pElement.remaining_capacity)
        # path state
        self.node_seq = []
        self.node_seq = copy.copy(pElement.node_seq)
        self.node_serving_state = []
        self.node_serving_state = copy.copy(pElement.node_serving_state)
        # path costs
        self.Primal_Label_cost = copy.copy(pElement.Primal_Label_cost)
        self.Label_cost_for_lagrangian = copy.copy(pElement.Label_cost_for_lagrangian)
        self.Label_cost_for_searching = copy.copy(pElement.Label_cost_for_searching)

    def Calculate_Label_Cost(self, node, link):
        self.Primal_Label_cost += link.distance
        self.Label_cost_for_lagrangian += link.distance + node.base_profit_for_lagrangian
        self.Label_cost_for_searching += link.distance + node.base_profit_for_searching

class C_time_indexed_state_vector:
    def __init__(self):
        self.current_time= 0
        self.VSStateVector=[]
        self.state_map=[]

    def Reset(self):
        self.current_time = 0
        self.VSStateVector = []
        self.state_map = []

    def m_find_state_index(self, string_key):  # tring_key?????????
        if string_key in self.state_map:
            return self.state_map.index(string_key)
        else:
            return -1

    def update_state(self, element, Flag):
        string_key = element.generate_string_key()
        state_index = self.m_find_state_index(string_key)
        if state_index == -1:
            self.VSStateVector.append(element)
            self.state_map.append(string_key)
        else:
            # Flag=1,ADMM?????????Flag=2,LR??????
            if Flag == 2:
                if element.Label_cost_for_lagrangian < self.VSStateVector[state_index].Label_cost_for_lagrangian:
                    self.VSStateVector[state_index] = element
            if Flag == 1:
                if element.Label_cost_for_searching < self.VSStateVector[state_index].Label_cost_for_searching:
                    self.VSStateVector[state_index] = element

    def Sort(self, Flag):
        if Flag == 1:
            self.VSStateVector = sorted(self.VSStateVector, key=lambda x: x.Label_cost_for_searching)
        if Flag == 2:
            self.VSStateVector = sorted(self.VSStateVector, key=lambda x: x.Label_cost_for_lagrangian)