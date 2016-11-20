/*
 * This source file is part of libRocket, the HTML/CSS Interface Middleware
 *
 * For the latest information, see http://www.librocket.com
 *
 * Copyright (c) 2008-2010 Nuno Silva
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/*
 * Modifed 2013 by Megavolt2013 in order to get the SFML2 sample working
 * with the revised libraries of SFML2
 * Please check the comments starting with NOTE in the files "main.cpp" and
 * "RenderInterfaceSFML.h" if you have trouble building this sample
 */

// NOTE: uncomment this only when you want to use the
// OpenGL Extension Wrangler Library (GLEW)
//#include <GL/glew.h>

#include <Rocket/Core.h>
#include "SystemInterfaceSFML.h"
#include "RenderInterfaceSFML.h"
#include <Rocket/Core/Input.h>
#include <Rocket/Debugger/Debugger.h>
#include "ShellFileInterface.h"
#include <cmath>
#include <unordered_set>
#include <sstream>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <entityx/entityx.h>

using std::cerr;
using std::cout;
using std::endl;

namespace ex = entityx;


float r(int a, float b = 0) {
    return static_cast<float>(std::rand() % (a * 1000) + b * 1000) / 1000.0;
}


struct Body {
    Body(const sf::Vector2f &position, const sf::Vector2f &direction, float rotationd = 0.0)
    : position(position), direction(direction), rotationd(rotationd) {}
    
    sf::Vector2f position;
    sf::Vector2f direction;
    float rotation = 0.0, rotationd;
};


struct Renderable {
    explicit Renderable(std::shared_ptr<sf::Shape> shape) : shape(std::move(shape)) {}
    
    std::shared_ptr<sf::Shape> shape;
};


struct Particle {
    explicit Particle(sf::Color colour, float radius, float duration)
    : colour(colour), radius(radius), alpha(colour.a), d(colour.a / duration) {}
    
    sf::Color colour;
    float radius, alpha, d;
};


struct Collideable {
    explicit Collideable(float radius) : radius(radius) {}
    
    float radius;
};


// Emitted when two entities collide.
struct CollisionEvent {
    CollisionEvent(ex::Entity left, ex::Entity right) : left(left), right(right) {}
    
    ex::Entity left, right;
};


class SpawnSystem : public ex::System<SpawnSystem> {
public:
    explicit SpawnSystem(sf::RenderTarget &target, int count) : size(target.getSize()), count(count) {}
    
    void update(ex::EntityManager &es, ex::EventManager &events, ex::TimeDelta dt) override {
        int c = 0;
        ex::ComponentHandle<Collideable> collideable;
        es.each<Collideable>([&](ex::Entity entity, Collideable&) { ++c; });
        
        for (int i = 0; i < count - c; i++) {
            ex::Entity entity = es.create();
            
            // Mark as collideable (explosion particles will not be collideable).
            collideable = entity.assign<Collideable>(r(10, 5));
            
            // "Physical" attributes.
            entity.assign<Body>(
                                sf::Vector2f(r(size.x), r(size.y)),
                                sf::Vector2f(r(100, -50), r(100, -50)));
            
            // Shape to apply to entity.
            std::shared_ptr<sf::Shape> shape(new sf::CircleShape(collideable->radius));
            shape->setFillColor(sf::Color(r(128, 127), r(128, 127), r(128, 127)));
            shape->setOrigin(collideable->radius, collideable->radius);
            entity.assign<Renderable>(std::move(shape));
        }
    }
    
private:
    sf::Vector2u size;
    int count;
};


// Updates a body's position and rotation.
struct BodySystem : public ex::System<BodySystem> {
    void update(ex::EntityManager &es, ex::EventManager &events, ex::TimeDelta dt) override {
        es.each<Body>([dt](ex::Entity entity, Body &body) {
            body.position += body.direction * static_cast<float>(dt);
            body.rotation += body.rotationd * dt;
        });
    };
};


// Bounce bodies off the edge of the screen.
class BounceSystem : public ex::System<BounceSystem> {
public:
    explicit BounceSystem(sf::RenderTarget &target) : size(target.getSize()) {}
    
    void update(ex::EntityManager &es, ex::EventManager &events, ex::TimeDelta dt) override {
        es.each<Body>([this](ex::Entity entity, Body &body) {
            if (body.position.x + body.direction.x < 0 ||
                body.position.x + body.direction.x >= size.x)
                body.direction.x = -body.direction.x;
            if (body.position.y + body.direction.y < 0 ||
                body.position.y + body.direction.y >= size.y)
                body.direction.y = -body.direction.y;
        });
    }
    
private:
    sf::Vector2u size;
};


// Determines if two Collideable bodies have collided. If they have it emits a
// CollisionEvent. This is used by ExplosionSystem to create explosion
// particles, but it could be used by a SoundSystem to play an explosion
// sound, etc..
//
// Uses a fairly rudimentary 2D partition system, but performs reasonably well.
class CollisionSystem : public ex::System<CollisionSystem> {
    static const int PARTITIONS = 200;
    
    struct Candidate {
        sf::Vector2f position;
        float radius;
        ex::Entity entity;
    };
    
public:
    explicit CollisionSystem(sf::RenderTarget &target) : size(target.getSize()) {
        size.x = size.x / PARTITIONS + 1;
        size.y = size.y / PARTITIONS + 1;
    }
    
    void update(ex::EntityManager &es, ex::EventManager &events, ex::TimeDelta dt) override {
        reset();
        collect(es);
        collide(events);
    };
    
private:
    std::vector<std::vector<Candidate>> grid;
    sf::Vector2u size;
    
    void reset() {
        grid.clear();
        grid.resize(size.x * size.y);
    }
    
    void collect(ex::EntityManager &entities) {
        entities.each<Body, Collideable>([this](ex::Entity entity, Body &body, Collideable &collideable) {
            unsigned int
            left = static_cast<int>(body.position.x - collideable.radius) / PARTITIONS,
            top = static_cast<int>(body.position.y - collideable.radius) / PARTITIONS,
            right = static_cast<int>(body.position.x + collideable.radius) / PARTITIONS,
            bottom = static_cast<int>(body.position.y + collideable.radius) / PARTITIONS;
            Candidate candidate {body.position, collideable.radius, entity};
            unsigned int slots[4] = {
                left + top * size.x,
                right + top * size.x,
                left  + bottom * size.x,
                right + bottom * size.x,
            };
            grid[slots[0]].push_back(candidate);
            if (slots[0] != slots[1]) grid[slots[1]].push_back(candidate);
            if (slots[1] != slots[2]) grid[slots[2]].push_back(candidate);
            if (slots[2] != slots[3]) grid[slots[3]].push_back(candidate);
        });
    }
    
    void collide(ex::EventManager &events) {
        for (const std::vector<Candidate> &candidates : grid) {
            for (const Candidate &left : candidates) {
                for (const Candidate &right : candidates) {
                    if (left.entity == right.entity) continue;
                    if (collided(left, right))
                        events.emit<CollisionEvent>(left.entity, right.entity);
                }
            }
        }
    }
    
    float length(const sf::Vector2f &v) {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }
    
    bool collided(const Candidate &left, const Candidate &right) {
        return length(left.position - right.position) < left.radius + right.radius;
    }
};


class ParticleSystem : public ex::System<ParticleSystem> {
public:
    void update(ex::EntityManager &es, ex::EventManager &events, ex::TimeDelta dt) override {
        es.each<Particle>([dt](ex::Entity entity, Particle &particle) {
            particle.alpha -= particle.d * dt;
            if (particle.alpha <= 0) {
                entity.destroy();
            } else {
                particle.colour.a = particle.alpha;
            }
        });
    }
};


class ParticleRenderSystem : public ex::System<ParticleRenderSystem> {
public:
    explicit ParticleRenderSystem(sf::RenderTarget &target) : target(target) {}
    
    void update(ex::EntityManager &es, ex::EventManager &events, ex::TimeDelta dt) override {
        sf::VertexArray vertices(sf::Quads);
        es.each<Particle, Body>([&vertices](ex::Entity entity, Particle &particle, Body &body) {
            const float r = particle.radius;
            vertices.append(sf::Vertex(body.position + sf::Vector2f(-r, -r), particle.colour));
            vertices.append(sf::Vertex(body.position + sf::Vector2f(r, -r), particle.colour));
            vertices.append(sf::Vertex(body.position + sf::Vector2f(r, r), particle.colour));
            vertices.append(sf::Vertex(body.position + sf::Vector2f(-r, r), particle.colour));
        });
        target.draw(vertices);
    }
private:
    sf::RenderTarget &target;
};


// For any two colliding bodies, destroys the bodies and emits a bunch of bodgy explosion particles.
class ExplosionSystem : public ex::System<ExplosionSystem>, public ex::Receiver<ExplosionSystem> {
public:
    void configure(ex::EventManager &events) override {
        events.subscribe<CollisionEvent>(*this);
    }
    
    void update(ex::EntityManager &es, ex::EventManager &events, ex::TimeDelta dt) override {
        for (ex::Entity entity : collided) {
            emit_particles(es, entity);
            entity.destroy();
        }
        collided.clear();
    }
    
    void emit_particles(ex::EntityManager &es, ex::Entity entity) {
        ex::ComponentHandle<Body> body = entity.component<Body>();
        ex::ComponentHandle<Renderable> renderable = entity.component<Renderable>();
        ex::ComponentHandle<Collideable> collideable = entity.component<Collideable>();
        sf::Color colour = renderable->shape->getFillColor();
        colour.a = 200;
        
        float area = (M_PI * collideable->radius * collideable->radius) / 3.0;
        for (int i = 0; i < area; i++) {
            ex::Entity particle = es.create();
            
            float rotationd = r(720, 180);
            if (std::rand() % 2 == 0) rotationd = -rotationd;
            
            float offset = r(collideable->radius, 1);
            float angle = r(360) * M_PI / 180.0;
            particle.assign<Body>(
                                  body->position + sf::Vector2f(offset * cos(angle), offset * sin(angle)),
                                  body->direction + sf::Vector2f(offset * 2 * cos(angle), offset * 2 * sin(angle)),
                                  rotationd);
            
            float radius = r(3, 1);
            particle.assign<Particle>(colour, radius, radius / 2);
        }
    }
    
    void receive(const CollisionEvent &collision) {
        // Events are immutable, so we can't destroy the entities here. We defer
        // the work until the update loop.
        collided.insert(collision.left);
        collided.insert(collision.right);
    }
    
private:
    std::unordered_set<ex::Entity> collided;
};


// Render all Renderable entities and draw some informational text.
class RenderSystem  :public ex::System<RenderSystem> {
public:
    explicit RenderSystem(sf::RenderTarget &target, sf::Font &font) : target(target) {
        text.setFont(font);
        text.setPosition(sf::Vector2f(2, 2));
        text.setCharacterSize(18);
        text.setFillColor(sf::Color::White);
    }
    
    void update(ex::EntityManager &es, ex::EventManager &events, ex::TimeDelta dt) override {
        es.each<Body, Renderable>([this](ex::Entity entity, Body &body, Renderable &renderable) {
            renderable.shape->setPosition(body.position);
            renderable.shape->setRotation(body.rotation);
            target.draw(*renderable.shape.get());
        });
        last_update += dt;
        frame_count++;
        if (last_update >= 0.5) {
            std::ostringstream out;
            const double fps = frame_count / last_update;
            out << es.size() << " entities (" << static_cast<int>(fps) << " fps)";
            text.setString(out.str());
            last_update = 0.0;
            frame_count = 0.0;
        }
        target.draw(text);
    }
    
private:
    double last_update = 0.0;
    double frame_count = 0.0;
    sf::RenderTarget &target;
    sf::Text text;
};


class Application : public ex::EntityX {
public:
    explicit Application(sf::RenderTarget &target, sf::Font &font) {
        systems.add<SpawnSystem>(target, 50);
        systems.add<BodySystem>();
        systems.add<BounceSystem>(target);
        systems.add<CollisionSystem>(target);
        systems.add<ExplosionSystem>();
        systems.add<ParticleSystem>();
        systems.add<RenderSystem>(target, font);
        systems.add<ParticleRenderSystem>(target);
        systems.configure();
    }
    
    void update(ex::TimeDelta dt) {
        systems.update_all(dt);
    }
};


int main(int argc, char **argv)
{
#ifdef ROCKET_PLATFORM_WIN32
        DoAllocConsole();
#endif

    int window_width = 800;
    int window_height = 600;
    std::srand(std::time(nullptr));
    
	sf::RenderWindow MyWindow(sf::VideoMode(window_width, window_height), "libRocket with SFML2", sf::Style::Close);
	MyWindow.setVerticalSyncEnabled(true);
    
#ifdef ENABLE_GLEW
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
	  /* Problem: glewInit failed, something is seriously wrong. */
	  fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
	  //...
	}
	fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
#endif
    sf::Font font;
    if (!font.loadFromFile("LiberationSans-Regular.ttf")) {
        cerr << "error: failed to load LiberationSans-Regular.ttf" << endl;
        return 1;
    }
    
    
	RocketSFMLRenderer Renderer;
	RocketSFMLSystemInterface SystemInterface;

	// NOTE: if fonts and rml are not found you'll probably have to adjust
	// the path information in the string
	ShellFileInterface FileInterface("./");

	if(!MyWindow.isOpen())
		return 1;

	Renderer.SetWindow(&MyWindow);
    
	Rocket::Core::SetFileInterface(&FileInterface);
	Rocket::Core::SetRenderInterface(&Renderer);
	Rocket::Core::SetSystemInterface(&SystemInterface);


	if(!Rocket::Core::Initialise())
		return 1;

	Rocket::Core::FontDatabase::LoadFontFace("Delicious-Bold.otf");
	Rocket::Core::FontDatabase::LoadFontFace("Delicious-BoldItalic.otf");
	Rocket::Core::FontDatabase::LoadFontFace("Delicious-Italic.otf");
	Rocket::Core::FontDatabase::LoadFontFace("Delicious-Roman.otf");

	Rocket::Core::Context *Context = Rocket::Core::CreateContext("default",
		Rocket::Core::Vector2i(MyWindow.getSize().x, MyWindow.getSize().y));

	Rocket::Debugger::Initialise(Context);

	Rocket::Core::ElementDocument *Document = Context->LoadDocument("demo.rml");
  //  Rocket::Core::Factory::RegisterEventListenerInstancer(
                
	if(Document)
	{
		Document->Show();
		Document->RemoveReference();
		fprintf(stdout, "\nDocument loaded");
	}
	else
	{
		fprintf(stdout, "\nDocument is NULL");
	}
    Application app(MyWindow, font);
    sf::Clock clock;
	while(MyWindow.isOpen())
	{
		static sf::Event event;
        MyWindow.clear();
        
        sf::Time elapsed = clock.restart();
        
        
        MyWindow.pushGLStates();
        glDisable(GL_SCISSOR_TEST);
        glDisable(GL_DEPTH_TEST);
      //  glDisable(GL_BLEND);
        Renderer.GetWindow()->setView(Renderer.GetWindow()->getDefaultView());
		app.update(elapsed.asSeconds());
        MyWindow.popGLStates();
        
        MyWindow.pushGLStates();
        Renderer.Resize();
        Context->Render();
        MyWindow.popGLStates();
        
        
        MyWindow.display();

		while(MyWindow.pollEvent(event))
		{
			switch(event.type)
			{
			case sf::Event::Resized:
				Renderer.Resize();
				break;
			case sf::Event::MouseMoved:
				Context->ProcessMouseMove(event.mouseMove.x, event.mouseMove.y,
					SystemInterface.GetKeyModifiers(&MyWindow));
				break;
			case sf::Event::MouseButtonPressed:
				Context->ProcessMouseButtonDown(event.mouseButton.button,
					SystemInterface.GetKeyModifiers(&MyWindow));
				break;
			case sf::Event::MouseButtonReleased:
				Context->ProcessMouseButtonUp(event.mouseButton.button,
					SystemInterface.GetKeyModifiers(&MyWindow));
				break;
			case sf::Event::MouseWheelMoved:
				Context->ProcessMouseWheel(-event.mouseWheel.delta,
					SystemInterface.GetKeyModifiers(&MyWindow));
				break;
			case sf::Event::TextEntered:
				if (event.text.unicode > 32)
					Context->ProcessTextInput(event.text.unicode);
				break;
			case sf::Event::KeyPressed:
				Context->ProcessKeyDown(SystemInterface.TranslateKey(event.key.code),
					SystemInterface.GetKeyModifiers(&MyWindow));
				break;
			case sf::Event::KeyReleased:
				if(event.key.code == sf::Keyboard::F8)
				{
					Rocket::Debugger::SetVisible(!Rocket::Debugger::IsVisible());
				};

				if(event.key.code == sf::Keyboard::Escape) {
					MyWindow.close();
				}

				Context->ProcessKeyUp(SystemInterface.TranslateKey(event.key.code),
					SystemInterface.GetKeyModifiers(&MyWindow));
				break;
			case sf::Event::Closed:
				MyWindow.close();
				break;
			};
		};
		Context->Update();
	};

	Context->RemoveReference();
	Rocket::Core::Shutdown();

	return 0;
};
