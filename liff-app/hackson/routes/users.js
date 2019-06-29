var express = require('express');
var router = express.Router();

// postgreSql 接続
var pg = require('pg');
// エラー一時回避 https://github.com/sequelize/sequelize/issues/3781
delete pg.native;
// 接続用文字列
// var conString = "postgres://postgres:131518gundam@localhost/chaos";
var conString = "postgres://postgres:moedra@localhost:5432/tebura";

// 接続リソース
var client = new pg.Client(conString);
/* GET users listing. */

router.get('/', function(req, res, next) {
  // view側、posgreSqlは result.rows.length、rowsを使う必要がある。
  client.connect(function(err) {
    client.query('select * from score order by score desc;', function(err, result) {
      console.log(result);
      res.render('users', { result: result});
    });
  });
});

router.get('/update', function(req, res, next) {
  // view側、posgreSqlは result.rows.length、rowsを使う必要がある。
  client.connect(function(err) {
    console.log(req.query.user_id);
    console.log(req.query.user_name);
    // ユーザーID
    var user_id = req.query.user_id;
    // ユーザーネーム
    var user_name = req.query.user_name;
    // 2019/06/09 まずはuserIDを検索、すでに登録された場合、score+1
    client.query("select * from score where user_id='" + user_id + "';", function(err, result) {
      // 2019/06/09 該当カラムある場合、result.rows.length > 0
      if (result.rows.length) {
        // そのユーザーIDの持ち主のscore+１
        client.query("update score set score= score + 1 where user_id='" + user_id + "';");
      } else {
        // 新規データとして登録
        client.query("insert into score (user_id, user_name, score) values ('" + user_id + "', '" + user_name + "', 1);");
      }
      res.redirect('../');
    });
  });
});

module.exports = router;
