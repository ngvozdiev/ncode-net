#include <stddef.h>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "ncode_common/src/common.h"
#include "ncode_common/src/strutil.h"
#include "ncode_common/src/logging.h"
#include "ncode_common/src/strutil.h"

using namespace std;

enum Symbol { LPAREN, RPAREN, AND, OR, THEN, FALLBACK, SET_ID };

class TreeNode {
 public:
  TreeNode(Symbol symbol) : symbol_(symbol) {}

  Symbol symbol() const { return symbol_; }

 private:
  Symbol symbol_;
};

class TerminalNode : public TreeNode {
 public:
  TerminalNode(const std::string& set_id, bool avoids)
      : TreeNode(SET_ID), set_id_(set_id), avoids_(avoids) {}

  // Identifies the graph evlements this terminal represents.
  const std::string& set_id() const { return set_id_; }

  // True if the graph elements identified by 'set_id' should be avoided. False
  // if they should be visited.
  bool avoids() const { return avoids_; }

 private:
  std::string set_id_;
  bool avoids_;
};

class ExpressionNode : public TreeNode {
 public:
  ExpressionNode(Symbol symbol, std::unique_ptr<TreeNode> left,
                 std::unique_ptr<TreeNode> right)
      : TreeNode(symbol), left_(std::move(left)), right_(std::move(right)) {}

 private:
  std::unique_ptr<TreeNode> left_;
  std::unique_ptr<TreeNode> right_;
};

// Splits an expression into symbols and extracts the set ids. Everything which
// is not a symbol is assumed to be a set id.
bool Tokenize(const std::string& expression, std::vector<Symbol>* symbols,
              std::vector<std::unique_ptr<TerminalNode>>* terminals) {
  std::string expression_braces_apart =
      nc::StringReplace(expression, "(", " ( ", true);
  expression_braces_apart =
      nc::StringReplace(expression_braces_apart, ")", " ) ", true);

  std::vector<std::string> split = nc::Split(expression_braces_apart, " ");
  for (size_t i = 0; i < split.size(); ++i) {
    std::string token = split[i];
    if (token == "(") {
      symbols->emplace_back(LPAREN);
    } else if (token == ")") {
      symbols->emplace_back(RPAREN);
    } else if (token == "then") {
      symbols->emplace_back(THEN);
    } else if (token == "or") {
      symbols->emplace_back(OR);
    } else if (token == "and") {
      symbols->emplace_back(AND);
    } else if (token == "visit") {
      if (i + 1 == split.size()) {
        LOG(ERROR) << "Unable to tokenize: cannot find id of elements to visit";
        return false;
      }

      terminals->emplace_back(split[++i], false);
    } else if (token == "avoid") {
      if (i + 1 == split.size()) {
        LOG(ERROR) << "Unable to tokenize: cannot find id of elements to avoid";
        return false;
      }

      terminals->emplace_back(split[++i], true);
    } else {
      LOG(ERROR) << "Unable to tokenize: unexpected token " << token;
      return false;
    }
  }

  return true;
}

class Parser {
 public:
  Parser(const std::vector<Symbol>& symbols,
         std::vector<std::unique_ptr<TerminalNode>>&& terminals)
      : symbol_index_(0),
        terminal_index_(-1),
        symbols_(symbols),
        terminals_(std::move(terminals)) {
    CHECK(!symbols_.empty());
    if (symbols_[0] == SET_ID) {
      ++terminal_index_;
      CHECK(terminals_.size() > terminal_index_);
    }
  }

  std::unique_ptr<TreeNode> Parse() {
    auto to_return = Then();
    CHECK(symbol_index_ == symbols_.size());
    CHECK(to_return);

    return to_return;
  }

 private:
  void NextSymbol() {
    ++symbol_index_;

    if (symbols_[symbol_index_] == SET_ID) {
      ++terminal_index_;
      CHECK(terminals_.size() > terminal_index_);
    }
  }

  bool Accept(Symbol s) {
    if (symbol_index_ == symbols_.size()) {
      return false;
    }

    if (symbols_[symbol_index_] == s) {
      NextSymbol();
      return true;
    }

    return false;
  }

  void Expect(Symbol s) {
    if (Accept(s)) {
      return;
    }

    LOG(FATAL) << "Unexpected symbol";
  }

  std::unique_ptr<TreeNode> P() {
    if (Accept(SET_ID)) {
      return std::move(terminals_[terminal_index_]);
    }

    if (Accept(LPAREN)) {
      auto then_return = Then();
      Expect(RPAREN);
      return std::move(then_return);
    }

    LOG(FATAL) << "Bad terminal";
    return {};
  }

  std::unique_ptr<TreeNode> Then() {
    auto lhs = P();
    CHECK(lhs != nullptr);

    auto rhs = ThenPrime();
    if (!rhs) {
      return lhs;
    }

    auto to_return = nc::make_unique<TreeNode>();
    to_return->op = THEN;
    to_return->left = std::move(lhs);
    to_return->right = std::move(rhs);
    return to_return;
  }

  std::unique_ptr<TreeNode> ThenPrime() {
    if (Accept(THEN)) {
      return Then();
    }

    return {};
  }

  size_t symbol_index_;
  size_t terminal_index_;
  std::vector<Symbol> symbols_;
  std::vector<std::unique_ptr<TerminalNode>> terminals_;
};

int main(void) {
  // (2) Make a parser
  auto syntax = R"(
        Then   <- Then 'then' P | P
        P      <- '(' Then ')' | id

        Then <- P Then'
        Then' <- 'then' P Then' | nil
        P <- '(' Then ')' | id
    )";

  std::vector<Symbol> symbols;
  std::vector<std::string> ids;
  std::tie(symbols, ids) = Tokenize("A then B then C");

  Parser p(symbols, ids);
  auto out = p.Parse();
  LOG(ERROR) << out->ToString();
}
