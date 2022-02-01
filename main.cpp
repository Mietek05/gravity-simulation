#include <SFML/Graphics.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <unordered_set>
#include <set>

using namespace std::literals::chrono_literals;

int SCREENWIDTH = 1000, SCREENHEIGHT = 618;

float framerate = 60;
float idealUpdateTime = 1/framerate;
float deltaTime = idealUpdateTime;
float timeScale = 1;

//float bodyScale = 100;
float mapScale = 1;
float camSpeed = 20;

const float GRAV = 6.674; // 6.674

struct Vec2
{
    float x, y;
    Vec2() {}
    Vec2(float _x, float _y) : x(_x), y(_y) {}
};

Vec2 camPos(0, 0);

// Magnitude axis.
struct MAxis
{
    float x, y;

    MAxis() {}
    MAxis(float _x, float _y) : x(_x), y(_y) {}
};

struct Force
{
    static int currentID;
    std::string ID;
    float x, y;
    // Number of frames it applies the force. -1 indicates indefinite application.
    int onetime;
    Force() {}
    Force(std::string _ID, float _x, float _y, int _onetime) : ID(_ID), x(_x), y(_y), onetime(_onetime) {}
};
int Force::currentID = 0;

float earthGravity = 1;
float worldMass = 59720000000000;

struct Body
{
    private:
        static void ForceCalc(Body *b)
        {
            b->generalForce->x = 0; b->generalForce->y = 0;
            if (!b->forces.empty())
            {
                for (auto f = b->forces.begin(); f != b->forces.end();)
                {
                    b->generalForce->x += (*f).second->x;
                    b->generalForce->y += (*f).second->y;
                    if ((*f).second->onetime > -1)
                    {
                        if ((*f).second->onetime > 0)
                            (*f).second->onetime--;
                        else
                            b->forces.erase(f++);
                    }
                    else
                        ++f;
                }
            }
            b->acc.x = b->generalForce->x/b->mass;
            b->acc.y = b->generalForce->y/b->mass;
        }
        static void AccelerationCalc(Body *b)
        {
            b->vel.x += b->acc.x*deltaTime*timeScale;
            b->vel.y += b->acc.y*deltaTime*timeScale;
        }
        static void VelocityCalc(Body *b)
        {
            b->pos.x += b->vel.x*deltaTime*timeScale;
            b->pos.y += b->vel.y*deltaTime*timeScale;
        }
        static void GravityCalc(Body *b1)
        {
            for (auto& b2 : bodies)
            {
                if (b2 != b1)
                {
                    if (b1->forces.find(b2->ID) == b1->forces.end())
                    {
                        Force* f = new Force(b2->ID, 0, 0, -1);
                        b1->forces.insert(std::pair<std::string, Force*>(b2->ID, f));
                    }
                    float massProduct = b1->mass*b2->mass;
                    float dX = b1->pos.x-b2->pos.x;
                    float dY = b1->pos.y-b2->pos.y;
                    float d = sqrtf(dX*dX + dY*dY);
                    if (d > 0)
                    {
                        float gravityForce = (GRAV*massProduct)/(d*d);
                        b1->forces[b2->ID]->x = (-dX/d)*gravityForce;
                        b1->forces[b2->ID]->y = (-dY/d)*gravityForce;
                    }
                }
            }  
        }
        static void CollisionDetection(Body *b1)
            {
                for (auto& b2 : collidingBodies)
                {
                    if (b2 != b1)
                    {
                        float touchingDistance = (b1->rad+b2->rad)*(b1->rad+b2->rad);
                        float difX = b1->pos.x-b2->pos.x;
                        float difY = b1->pos.y-b2->pos.y;
                        float distance = sqrtf(difX*difX+difY*difY);
                        if (distance*distance <= touchingDistance)
                        {
                            float overlap = 0.5*(distance-b1->rad-b2->rad); // 0.5 is going to detect the collision twice.
                            float nX = difX/distance;
                            float nY = difY/distance;
                            b1->pos.x -= nX*overlap;
                            b1->pos.y -= nY*overlap;
                            b2->pos.x += nX*overlap;
                            b2->pos.y += nY*overlap;
                            float velX = b1->vel.x;
                            float velY = b1->vel.y;
                            if (b1->kinetic)
                            {
                                // The first body takes the velocity of the second body, changes the direction to the bodies' position difference vector and changes the magnitude according to the mass.
                                b1->vel.x = b2->vel.x*-nX*b2->mass/b1->mass;
                                b1->vel.y = b2->vel.y*-nY*b2->mass/b1->mass;
                                //b1->acc.x *= -nX;
                                //b1->acc.y *= -nY;
                                // Old idea using the applying force on the bodies.
                                //b1->ForceOn(b2->ID+"COL", , b2->generalForce->y, true);
                            }
                            if (b2->kinetic)
                            {
                                b2->vel.x = velX*-nX*b1->mass/b2->mass;
                                b2->vel.y = velY*-nY*b1->mass/b2->mass;
                                //b2->ForceOn(b1->ID+"COL", b1->generalForce->x, b2->generalForce->y, true);
                            }
                        } 
                    }
                }
            }
            static void CollisionFinding()
            {
                collidingBodies.clear();
                for (auto& b1 : bodies)
                {
                    bool inserted = false;
                    for (auto& b2 : bodies)
                    {
                        if (b1 != b2)
                        {
                            if (b1->pos.x-b1->rad <= b2->pos.x+b2->rad && b2->pos.x-b2->rad <= b1->pos.x+b1->rad)
                            {
                                collidingBodies.insert(b1);
                                collidingBodies.insert(b2);
                                inserted = true;
                            }   
                        }
                    }
                    if (inserted)
                        CollisionDetection(b1);
                }
            }

    public:
        static std::unordered_set<Body*> bodies;
        static std::unordered_set<Body*> collidingBodies;
        std::map<std::string, Force*> forces;
        std::string ID;
        float mass, rad;
        Vec2 pos;
        MAxis vel, acc;
        bool gravitating, earthGravitating, kinetic;
        Force *generalForce;
        sf::CircleShape boxShape;

        Body(std::string _ID, int _mass = 0, Vec2 _pos = Vec2(0, 0))
        {
            bodies.insert(this);
            ID = _ID;
            mass = _mass;
            rad = mass;
            pos = _pos;
            vel = MAxis(0, 0);
            acc = MAxis(0, 0);
            gravitating = true;
            earthGravitating = false;
            kinetic = true;
            generalForce = new Force("GENERAL_FORCE", 0, 0, -1);
            boxShape.setRadius(rad);
            //boxShape.setSize(sf::Vector2f(bodyScale, bodyScale));
            boxShape.setOrigin(rad, rad);
            boxShape.setFillColor(sf::Color::Green);
        }

        ~Body()
        {
            bodies.erase(this);
        }
        void ForceOn(std::string ID, float x, float y, int im)
        {
            Force *f = new Force(ID, x, y, im);
            forces.insert(std::pair<std::string, Force*>(ID, f));
        }
        static void Calculations()
        {
            CollisionFinding();
            for (auto& b : bodies)
            {   
                //CollisionDetection(b); Old collisions.
                if (b->gravitating)
                    GravityCalc(b);
                ForceCalc(b);
                if (b->earthGravitating)
                    b->acc.y+=earthGravity;
                AccelerationCalc(b);
                VelocityCalc(b);
                b->boxShape.setPosition(b->pos.x*mapScale+camPos.x, b->pos.y*mapScale+camPos.y);
            }
        }
};
std::unordered_set<Body*> Body::bodies;
std::unordered_set<Body*> Body::collidingBodies;


int main()
{
    sf::RenderWindow window(sf::VideoMode(SCREENWIDTH, SCREENHEIGHT), "Physics");

    Body box1("BOX1", 60, Vec2(100, 100));
    Body box2("BOX2", 50, Vec2(100, SCREENHEIGHT-100));
    Body box3("BOX3", 40, Vec2(SCREENWIDTH-300, 200));
    Body box6("BOX6", 50, Vec2(SCREENWIDTH/2, 500));
    Body box7("BOX7", 60, Vec2(SCREENWIDTH-60, 400));
    Body box8("BOX8", 50, Vec2(SCREENWIDTH/2, 50));
    Body box9("BOX9", 40, Vec2(300, SCREENHEIGHT/2));

    sf::Font font;
    if (!font.loadFromFile("Arimo-Bold.ttf"))
        std::cout << "Error loading font." << std::endl;
    sf::Text framerateText;
    framerateText.setFont(font);
    framerateText.setString("FPS");
    framerateText.setCharacterSize(24);
    framerateText.setFillColor(sf::Color::Red);

    while (window.isOpen())
    {
        auto updateStart = std::chrono::steady_clock::now();
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            else if (event.type == sf::Event::KeyPressed)
            {
                switch (event.key.code)
                {
                    case sf::Keyboard::Left:
                        camPos.x += camSpeed;
                        break;
                    case sf::Keyboard::Right:
                        camPos.x -= camSpeed;
                        break;
                    case sf::Keyboard::Up:
                        camPos.y += camSpeed;
                        break;
                    case sf::Keyboard::Down:
                        camPos.y -= camSpeed;
                        break;
                }
            }
        }
        Body::Calculations();
        // Display.
        window.clear();
        for (auto& b : Body::bodies)
        {
            window.draw(b->boxShape);
        }
        window.draw(framerateText);
        auto updateEnd = std::chrono::steady_clock::now();
        std::chrono::duration<float, std::micro> updateTimeUsed = updateEnd - updateStart;
        deltaTime = updateTimeUsed.count()*0.0001;
        framerateText.setString("FPS: " + std::to_string(1/deltaTime));
        // 0,0167 - 0,0020
        //std::this_thread::sleep_for(std::chrono::microseconds(1670) - updateTimeUsed);
        window.display();
    }

    return 0;
}