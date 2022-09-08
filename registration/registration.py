class Registration:
    # slicer_buffer = ""

    def __init__(self):
        self.s_points = [[[], [], []],
                    [[], [], []],
                    [[], [], []],
                    [[], [], []]]

        # a fazer: do jeito que esta atualmente, os pontos tem que estar na ordem correta, pensar em uma soluçao para contorar ou avisar o 
        # utilizador que tal transformaçao nao é valida

        self.r_points = [[[], [], []],
                        [[], [], []],
                        [[], [], []],
                        [[], [], []]]
        
    # def show_points(self, type):
    #     if type == 1:
    #         print(self.s_points)
    #     else:
    #         print(self.r_points)

    def parsing(self, buffer_p):
        # print(buffer[2])
        # print(buffer[2].split())

        parsed_p = buffer_p

        for i in range(4): # setando para os 4 pontos fiduciais
            self.s_points[i][0] = parsed_p[i].split()[0] # setando x
            self.s_points[i][1] = parsed_p[i].split()[1] # y
            self.s_points[i][2] = parsed_p[i].split()[2] # z

        for i in range(4):
            if self.s_points[i][0] == "" or self.s_points[i][1] == "" or self.s_points[i][2] == "":
                print("ERRO: Pontos não foram corretamente preenchidos: \n")
                print(self.s_points)
                return False
        
        print("Pontos ajustaos e adicionado a lista da classe. \nFinal:\n")
        print(self.s_points)
        return True

    def slicer_read(self):
        with open("slicer_points.txt", "r") as file:
            # slicer_buffer = [[''], [''], [''], ['']]
            try: 
                slicer_buffer = file.readlines()
            except Exception as e:
                print(f"ERRO: Não foi possível ler do arquivo \'slicer_points.txt\'. {e}")
                return False
            

            for i in range(4):
                if slicer_buffer[i][-2] != f'{i+1}':
                    print(slicer_buffer[i][-2])
                    print(f"WARNING: Linha de index {i} não foi lida.")
                    return False
            
            if self.parsing(slicer_buffer):
                return True
            else:
                return False

    


def main():

    registration = Registration()
    print(f"operaçao retornou {registration.slicer_read()}")


if __name__ == "__main__":
    main()