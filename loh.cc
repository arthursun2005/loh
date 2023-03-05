// trans rights

#include <SFML/Graphics.hpp>
#include <fmt/core.h>

#include "group.h"

int main(int argc, const char *argv[])
{
    ios::sync_with_stdio(false);
    cin.tie(0);
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8.0f;
    sf::RenderWindow window(sf::VideoMode::getDesktopMode(), "loh", sf::Style::Fullscreen, settings);
    window.setVerticalSyncEnabled(true);

    FluidGroup group;
    vector<Vec2> shape {{1.0f, 0.0f}, {-0.5f, 0.87f}, {-0.5f, -0.87f}};

    sf::Font font;
    font.loadFromFile("/usr/local/share/fonts/Domigorgon.ttf");

    auto lt = now();
    int frames = 0;
    float fps = 0.0f;
    vector<sf::Vertex> vertices;
    bool paused = false;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Left)
                {
                    for (int i = -5; i <= 5; i++)
                        for (int j = -5; j <= 5; j++)
                            group.add(event.mouseButton.x + i * group.radius * 1.5f, event.mouseButton.y + j * group.radius * 1.5f);
                }
                if (event.mouseButton.button == sf::Mouse::Right)
                {
                    for (int i = -25; i <= 25; i++)
                        for (int j = -25; j <= 25; j++)
                            group.add(event.mouseButton.x + i * group.radius * 1.5f, event.mouseButton.y + j * group.radius * 1.5f);
                }
            }
            if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::Space)
                {
                    paused = not paused;
                }
            }
        }

        if (not paused)
        {
            int substeps = 8;
            for (int i = 0; i < substeps; i++)
            {
                float dt = 0.03f / (float) substeps;
                group.attract(window.getSize().x * 0.5f, window.getSize().y * 0.5f, dt * 0.2f);
                group.step(dt);
            }
        }

        window.clear();
        vertices.clear();
        for (size_t i = 0; i < group.pos.size(); i++)
        {
            Vec2 p = group.pos[i];
            float cl1 = tanh(group.vel[i].norm() * 0.01f);
            float cl2 = tanh(group.vel[i].norm() * 0.002f);
            sf::Color color(25.0f + 200.0f * cl2, 25.0f + 125.0f * cl1, 125.0f + 125.0f * cl1 - 80.0f * cl2);
            for (int j = 0; j < 3; j++)
            {
                Vec2 d = p + group.radius * 0.75f * shape[j];
                vertices.push_back(sf::Vertex({d.x(), d.y()}, color));
            }
        }
        window.draw(vertices.data(), vertices.size(), sf::Triangles);
        sf::Text fps_label(fmt::format("{} fps: {:.1f}", group.idx.size(), fps), font, 20);
        window.draw(fps_label);
        window.display();

        frames++;
        float dur = chrono::duration<float>(now() - lt).count();
        fps = frames / dur;
        if (dur > 1.0f)
        {
            lt = now();
            frames = 0;
        }
    }
    return 0;
}
