#include "search_progress.h"


SearchProgress::SearchProgress() {
    expanded_states = 0;
    reopened_states = 0;
    evaluated_states = 0;
    generated_states = 0;
    generated_ops = 0;

    lastjump_expanded_states = 0;
    lastjump_reopened_states = 0;
    lastjump_evaluated_states = 0;
    lastjump_generated_states = 0;

    lastjump_f_value = -1;
}

SearchProgress::~SearchProgress() {
}

void SearchProgress::add_heuristic(Heuristic *h) {
    heuristics.push_back(h);
    best_heuristic_values.push_back(-1);
}

void SearchProgress::report_f_value(int f) {
    if (f > lastjump_f_value) {
        lastjump_f_value = f;
        print_f_line();
        lastjump_expanded_states = expanded_states;
        lastjump_reopened_states = reopened_states;
        lastjump_evaluated_states = evaluated_states;
        lastjump_generated_states = generated_states;
    }
}

void SearchProgress::get_initial_h_values() {
    for (unsigned int i = 0; i < heuristics.size(); i++) {
        initial_h_values.push_back(heuristics[i]->get_heuristic());
    }
}

bool SearchProgress::check_h_progress(int g, double p) {
    bool progress = false;
    for(int i = 0; i < heuristics.size(); i++) {
        if(heuristics[i]->is_dead_end())
            continue;
        int h = heuristics[i]->get_heuristic();
        int &best_h = best_heuristic_values[i];
        if(best_h == -1 || h < best_h) {
            best_h = h;
            progress = true;
        }
    }
    if (progress) {
        print_h_line(g, p);
    }
    return progress;
}

void SearchProgress::print_f_line() const {
    cout << "f = " << lastjump_f_value
             << " [";
    print_line();
    cout << "]" << endl;
}

void SearchProgress::print_h_line(int g, double p) const {
    cout << "Best heuristic value: ";
    for(int i = 0; i < heuristics.size(); i++) {
    cout << best_heuristic_values[i];
    if(i != heuristics.size() - 1)
        cout << "/";
    }
    cout << " [g=" << g << ", p=" << p << ", ";
    print_line();
    cout << "]" << endl;
}

void SearchProgress::print_line() const {
    cout << evaluated_states << " evaluated, "
         << expanded_states << " expanded, ";
    if(reopened_states > 0) {
        cout << reopened_states << " reopened, ";
    }
    cout << "t=" << g_timer;
}

void SearchProgress::print_statistics() const {
    cout << "Initial state h value: ";
    for(int i = 0; i < initial_h_values.size(); i++) {
        cout << initial_h_values[i];
        if(i != initial_h_values.size() - 1)
            cout << "/";
    }
    cout << "." << endl;

    cout << "Expanded " << expanded_states << " state(s)." << endl;
    cout << "Reopened " << reopened_states << " state(s)." << endl;
    cout << "Evaluated " << evaluated_states << " state(s)." << endl;
    cout << "Generated " << generated_states << " state(s)." << endl;

    if (lastjump_f_value >= 0) {
        cout << "Expanded until last jump: "
            << lastjump_expanded_states << " state(s)." << endl;
        cout << "Reopened until last jump: "
            << lastjump_reopened_states << " state(s)." << endl;
        cout << "Evaluated until last jump: "
            << lastjump_evaluated_states << " state(s)." << endl;
        cout << "Generated until last jump: "
            << lastjump_generated_states << " state(s)." << endl;
    }
}
