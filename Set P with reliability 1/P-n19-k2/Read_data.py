#Read the VRP file, and output the node and link list
import random
import sys
class Read_txt:
    def __init__(self,based_profit):
        self.based_profit=based_profit
        self.file="input.vrp"
        print("Reading data...")
    def read_nodes(self):
        with open(self.file,"r") as fl:
            lines=fl.readlines()
            line_3_list=lines[3].strip().split(":")
            self.g_number_of_customers=int(line_3_list[-1])

            line_5_list=lines[5].strip().split(":")
            self.capacity=int(line_5_list[-1])

            self.g_number_of_nodes=0
            self.node_list=[]
            #add departing depot
            self.depart_depot=self.g_number_of_nodes
            # read node coord
            for index in range(7,7+self.g_number_of_customers):
                line_list = lines[index].strip().split(" ")
                node=Node()
                node.node_id=self.g_number_of_nodes
                node.x=int(line_list[1])
                node.y=int(line_list[2])
                node.base_profit_for_searching=self.based_profit
                node.base_profit_for_lagrangian=self.based_profit
                self.node_list.append(node)
                self.g_number_of_nodes+=1
            #read demands
            for index in range(8+self.g_number_of_customers,8+2*self.g_number_of_customers):
                line_list = lines[index].strip().split(" ")
                node_id=int(line_list[0])
                demand = int(line_list[1])
                node=self.node_list[node_id-1]
                node.demand=demand

            #add ending depot
            self.ending_depot = self.g_number_of_nodes
            import copy
            node=copy.deepcopy(self.node_list[0])
            node.node_id = self.g_number_of_nodes
            self.node_list.append(node)
            self.g_number_of_nodes += 1
            print()

    def generate_links(self):
        self.read_nodes()
        self.link_list = []
        self.g_number_of_links = 0
        for from_node in range(0, self.g_number_of_nodes):
            for to_node in range(0, self.g_number_of_nodes):
                if from_node == to_node:
                    continue
                if from_node==self.g_number_of_nodes-1:
                    continue
                if to_node==0:
                    continue

                x_from_node = self.node_list[from_node].x
                y_from_node = self.node_list[from_node].y
                x_to_node = self.node_list[to_node].x
                y_to_node = self.node_list[to_node].y

                mean = ((x_from_node - x_to_node) ** 2 + (y_from_node - y_to_node) ** 2) ** 0.5

                link = Link()
                link.link_id = self.g_number_of_links
                link.from_node_id = from_node
                link.to_node_id = to_node

                link.mean = round(mean)
                self.link_list.append(link)

                self.node_list[from_node].outbound_nodes_list.append(to_node)
                self.node_list[from_node].outbound_links_list.append(link)
                self.node_list[from_node].outbound_size = len(self.node_list[from_node].outbound_links_list)

                self.g_number_of_links += 1
        #variance reading or generation
        try:
            with open("link.csv","r") as fl:
                lines=fl.readlines()
                for line in lines[1:]:
                    list=line.strip().split(",")
                    link_index=int(list[0])
                    link=self.link_list[link_index]
                    #check
                    if link.from_node_id==int(list[1]) and link.to_node_id==int(list[2]):
                        link.variance=int(list[3])
                    else:
                        print("Cannot read the variance!")
                        sys.exit()
                print("Variances are read!")
        except:
            for link in self.link_list:
                link.variance = round(link.mean * random.random(),2)
            self.output_file()
            print("variances are output in the link.csv")
        return self.node_list, self.link_list, self.g_number_of_nodes, self.capacity,self.depart_depot,self.ending_depot


    def output_file(self):
        with open("Node.csv", "w") as fl:
            fl.write("Node_id,x_coordinate,y_coordinate,demand\n")
            for node in self.node_list:
                node_id = node.node_id
                x_coordinate = node.x
                y_coordinate = node.y
                demand = node.demand
                fl.write(str(node_id) + "," + str(x_coordinate) + "," + str(y_coordinate) + "," + str(demand) + "\n")

        with open("link.csv", "w") as fl:
            fl.write("link_id,from_node,to_node,mean,variance\n")
            for link in self.link_list:
                link_id = link.link_id
                from_node = link.from_node_id
                to_node = link.to_node_id
                mean = link.mean
                var=link.variance
                fl.write(str(link_id) + "," + str(from_node) + "," + str(to_node) + "," + str(mean) + "," + str(var)+ "\n")
        """
        with open("GAMS.txt","w") as fl:
            #index
            fl.write("set v /1*{}/;\n".format(5))
            fl.write("set i node id /0*{}/;\n".format(self.g_number_of_nodes-1))
            fl.write("alias(i,j);\n")

            # parameters cost
            fl.write("parameter cost(i,j) ;\n")
            for link in self.link_list:
                arc_cost = link.mean
                if link.from_node_id==0 and link.to_node_id==self.g_number_of_nodes-1:
                    arc_cost=0.0001
                from_node = "'" + str(link.from_node_id) + "',"
                to_node = "'" + str(link.to_node_id) + "'"
                fl.write("cost(" + from_node + to_node + ")={};\n".format(arc_cost))

            # demand
            fl.write("Parameter demand(i);\n")
            for node_id in range(1, self.g_number_of_nodes-1):
                node = self.node_list[node_id]
                demand=node.demand
                node_str = "'" + str(node_id) + "'"
                fl.write("demand(" + node_str + ")={};\n".format(demand))

            # parameters-capacity
            fl.write("parameter Ca /{}/;\n".format(self.capacity))

            # flow balance
            fl.write("parameter origin_node(v,i); \n")
            fl.write("origin_node(v,'{}') = 1;\n".format(self.depart_depot))

            fl.write("parameter destination_node(v,i);\n")
            fl.write("destination_node(v,'{}') = 1;\n".format(self.ending_depot))

            fl.write("parameter intermediate_node(v,i);\n")
            fl.write("intermediate_node(v,i) = (1- origin_node(v,i))*(1- destination_node(v,i));\n")
        """

class Node:
    def __init__(self):
        self.node_id=None
        # self.node_type=None
        self.x=None
        self.y=None
        self.demand=None
        self.outbound_nodes_list=[]
        self.outbound_size=0
        self.outbound_links_list=[]
        self.base_profit_for_searching = None
        self.base_profit_for_lagrangian = None

class Link:
    def __init__(self):
        self.link_id=None
        self.link_type=None
        self.from_node_id=None
        self.to_node_id=None
        self.mean=None
        self.variance=None

# mod=Read_txt(0)
# mod.generate_links()
