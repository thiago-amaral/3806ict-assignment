// Sample code to generate world. The actual world is generated in cpp
// This needs to be modified, originally I had thought this would write the CSP code
// but all it needs to do is generate the "known" world according to the submarine.
// Once the known model of the world is formed, it gets sent to PAT, which then determines path to take.
// Consider this a shell/template that I used to try and generate a csp file.

#include <vector>
#include <iostream>
#include <random>
#include <fstream>

int VISITED = -1;
int UNVISITED = 0;
int SUB = 1;
int HOSTILE = 2;
int SURVIVOR = 3;

class World
{
public:
    int world_width;
    int world_height;
    int survivors;
    int hostiles;
    std::vector<std::vector<int>> world;

    World(int ww, int wh, int s, int h)
    {
        world_width = ww;
        world_height = wh;
        survivors = s;
        hostiles = h;

        // init grid to UNVISITED
        world = std::vector<std::vector<int>>(world_height, std::vector<int>(world_width, UNVISITED));

        // place sub
        world[world_height - 1][0] = SUB;

        // place survivors
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> rowDist(0, world_height - 1);
        std::uniform_int_distribution<int> colDist(0, world_width - 1);

        int placed = 0;
        while (placed < survivors)
        {
            int rand_row = rowDist(gen);
            int rand_col = colDist(gen);
            if (world[rand_row][rand_col] == UNVISITED)
            {
                world[rand_row][rand_col] = SURVIVOR;
                placed++;
            }
        }

        // place hostiles
        placed = 0;
        while (placed < hostiles)
        {
            int rand_row = rowDist(gen);
            int rand_col = colDist(gen);
            if (world[rand_row][rand_col] == UNVISITED)
            {
                world[rand_row][rand_col] = HOSTILE;
                placed++;
            }
        }
    }

    void write_world(std::string filename)
    {
        std::ofstream file(filename);
        if (!file)
        {
            std::cerr << "Failed to open the file: " << filename << std::endl;
            return;
        }
        file << "#define Visited -1;\n";
        file << "#define Unvisited 0;\n";
        file << "#define Sub 1;\n";
        file << "#define Bomb 2;\n";
        file << "#define Survivor 3;\n\n";
        file << "#define Rows " << world_height << ";\n";
        file << "#define Cols " << world_width << ";\n";
        file << "#define Fuel " << world_height * world_width << ";\n";
        file << "\n// this world:\n";

        for (auto row : world)
        {
            file << "// ";
            for (auto col : row)
            {
                file << col << " ";
            }
            file << "\n";
        }

        file << "\nvar world[Rows][Cols]:{Visited..Survivor} = [";
        for (int i = 0; i < world_height; i++)
        {
            for (int j = 0; j < world_width; j++)
            {
                if (i == world_height - 1 && j == world_width - 1)
                {
                    file << world[i][j];
                }
                else
                {
                    file << world[i][j] << ", ";
                }
            }
            file << "\n";
        }
        file << "];";

        file << "// Initial positions of sub";
        // KEEP DOING THIS CRAP HERE TO GENERATE STARTING WORLD
        // the world needs to know where the bombs are but the sub doesnt
        // until it gets close

        file.close();
        std::cout << "Initial environment created.\n"
                  << std::endl;
    }

    void print_world()
    {
        for (int i = 0; i < world_height; i++)
        {
            for (int j = 0; j < world_width; j++)
            {
                std::cout << world[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

    void send_world()
    {
        std_msgs::String row;
        row.data = "OOSOOS";

        for (int i = 0; i < 6; i++)
        {
            srv.request.grid.push_back(row);
        }

        if (!client.call(srv))
        {
            return EXIT_FAILURE;
        }
    }
};

int main()
{
    World my_world(6, 6, 3, 4);
    my_world.print_world();
    my_world.write_world("my_world.csp");
}
