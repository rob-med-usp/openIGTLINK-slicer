class teste():
    nome = "batata"
    def mudax(self, nome):
        self.nome = nome
        return self.nome

k=teste()
k2=teste()
print(k.mudax("teste"))
print(k2.mudax("teste2"))